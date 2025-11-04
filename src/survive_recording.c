// All MIT/x11 Licensed Code in this file may be relicensed freely under the GPL
// or LGPL licenses.
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include <survive.h>

#include <stdio.h>
#include <stdlib.h>
#include <survive.h>

#include <assert.h>
#include <errno.h>
#include <string.h>

#include <inttypes.h>

#include "survive_recording.h"

#include "survive_config.h"
#include "survive_default_devices.h"

#include "ctype.h"
#include "os_generic.h"
#include "stdarg.h"

#include "survive_gz.h"

// UDP streaming includes
#ifdef _WIN32
#include "winsock2.h"
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#endif

#ifndef MSG_NOSIGNAL
#define MSG_NOSIGNAL 0
#endif

typedef struct SurviveRecordingData {
	SurviveContext *ctx;
	bool alwaysWriteStdOut;
	bool writeRawLight;
	bool writeIMU;
	bool writeCalIMU;
	bool writeAngle;
	int writeDataMatrix;
	gzFile output_file;
	
	// UDP streaming fields
	bool udpStreamEnabled;
	int udp_socket;
	struct sockaddr_in udp_target_addr;
	char udp_buffer[8192];  // Buffer for UDP messages
} SurviveRecordingData;

// clang-format off
STRUCT_CONFIG_SECTION(SurviveRecordingData)
    STRUCT_CONFIG_ITEM("record-rawlight", "Whether or not to output raw light data", 1, t->writeRawLight)
    STRUCT_CONFIG_ITEM("record-imu", "Whether or not to output imu data", 1, t->writeIMU)
    STRUCT_CONFIG_ITEM("record-cal-imu", "Whether or not to output calibrated imu data", 0, t->writeCalIMU)
	STRUCT_CONFIG_ITEM("record-angle", "Whether or not to output angle data", 1, t->writeAngle)
	STRUCT_CONFIG_ITEM("record-data-matrices", "Whether or not to output data matrices", 0, t->writeDataMatrix)
	STRUCT_CONFIG_ITEM("udp-stream", "Enable UDP streaming of recording data", 0, t->udpStreamEnabled)
END_STRUCT_CONFIG_SECTION(SurviveRecordingData)
	// clang-format on

	STATIC_CONFIG_ITEM(RECORD, "record", 's', "File to record to if you wish to make a recording.", "")
	STATIC_CONFIG_ITEM(RECORD_STDOUT, "record-stdout", 'b', "Whether or not to dump recording data to stdout", 0)
	STATIC_CONFIG_ITEM(UDP_STREAM_HOST, "udp-stream-host", 's', "UDP target host for streaming", "127.0.0.1")
	STATIC_CONFIG_ITEM(UDP_STREAM_PORT, "udp-stream-port", 'i', "UDP target port for streaming", 2333)

// Forward declarations for UDP streaming functions
static void udp_stream_init(SurviveRecordingData *recordingData);
static void udp_stream_send(SurviveRecordingData *recordingData, const char *data, int len);
static void udp_stream_cleanup(SurviveRecordingData *recordingData);

	static void write_to_output_raw(SurviveRecordingData *recordingData, const char *string, int len) {
		if (recordingData->output_file) {
			gzwrite(recordingData->output_file, string, len);
		}

		if (recordingData->alwaysWriteStdOut) {
			fwrite(string, 1, len, stdout);
		}
}

#ifdef SURVIVE_HEX_FLOATS
#define FLT_PRINTF "%0.6a "
#else
#define FLT_PRINTF "%0.6f "
#endif

SURVIVE_EXPORT void survive_recording_write_matrix(struct SurviveRecordingData *recordingData, const SurviveObject *so,
												   int lvl, const char *name, const CnMat *M) {
	if (!recordingData || recordingData->writeDataMatrix < lvl || !M || M->rows == 0 || M->cols == 0) {
		return;
	}

	survive_recording_write_to_output(recordingData, "%s DATA_MATRIX %s %d %d ", so ? so->codename : "g", name, M->rows,
									  M->cols);
	for (int i = 0; i < M->rows * M->cols; i++) {
		survive_recording_write_to_output_nopreamble(recordingData, "%f ", M->data[i]);
	}
	survive_recording_write_to_output_nopreamble(recordingData, "\n");
}
void survive_recording_write_to_output(struct SurviveRecordingData *recordingData, const char *format, ...) {
	if (!recordingData) {
		return;
	}

	double ts = survive_run_time(recordingData->ctx);

	if (recordingData->output_file) {
		va_list args;
		va_start(args, format);
		gzprintf(recordingData->output_file, FLT_PRINTF, ts);
		gzvprintf(recordingData->output_file, format, args);

		va_end(args);
	}

	if (recordingData->alwaysWriteStdOut) {
		va_list args;
		va_start(args, format);
		fprintf(stdout, FLT_PRINTF, ts);
		vfprintf(stdout, format, args);
		va_end(args);
	}

	// UDP streaming
	if (recordingData->udpStreamEnabled) {
		va_list args;
		va_start(args, format);
		int len = snprintf(recordingData->udp_buffer, sizeof(recordingData->udp_buffer), FLT_PRINTF, ts);
		len += vsnprintf(recordingData->udp_buffer + len, sizeof(recordingData->udp_buffer) - len, format, args);
		va_end(args);
		udp_stream_send(recordingData, recordingData->udp_buffer, len);
	}
}

void survive_recording_write_to_output_nopreamble(struct SurviveRecordingData *recordingData, const char *format, ...) {
	if (!recordingData) {
		return;
	}

	if (recordingData->output_file) {
		va_list args;
		va_start(args, format);
		gzvprintf(recordingData->output_file, format, args);

		va_end(args);
	}

	if (recordingData->alwaysWriteStdOut) {
		va_list args;
		va_start(args, format);
		vfprintf(stdout, format, args);
		va_end(args);
	}

	// UDP streaming (no timestamp preamble)
	if (recordingData->udpStreamEnabled) {
		va_list args;
		va_start(args, format);
		int len = vsnprintf(recordingData->udp_buffer, sizeof(recordingData->udp_buffer), format, args);
		va_end(args);
		udp_stream_send(recordingData, recordingData->udp_buffer, len);
	}
}

// UDP streaming helper functions
static void udp_stream_init(SurviveRecordingData *recordingData) {
	if (!recordingData || !recordingData->udpStreamEnabled) {
		return;
	}

	SurviveContext *ctx = recordingData->ctx;
	recordingData->udp_socket = socket(AF_INET, SOCK_DGRAM, 0);
	if (recordingData->udp_socket < 0) {
		SV_WARN("Failed to create UDP socket for streaming");
		recordingData->udpStreamEnabled = false;
		return;
	}

	memset(&recordingData->udp_target_addr, 0, sizeof(recordingData->udp_target_addr));
	recordingData->udp_target_addr.sin_family = AF_INET;
	
	const char *host = survive_configs(ctx, UDP_STREAM_HOST_TAG, SC_GET, "127.0.0.1");
	int port = survive_configi(ctx, UDP_STREAM_PORT_TAG, SC_GET, 2333);
	
	if (inet_pton(AF_INET, host, &recordingData->udp_target_addr.sin_addr) <= 0) {
		SV_WARN("Invalid UDP stream host: %s", host);
		recordingData->udpStreamEnabled = false;
		close(recordingData->udp_socket);
		return;
	}
	
	recordingData->udp_target_addr.sin_port = htons(port);
	SV_INFO("UDP streaming enabled to %s:%d", host, port);
}

static void udp_stream_send(SurviveRecordingData *recordingData, const char *data, int len) {
	if (!recordingData || !recordingData->udpStreamEnabled || recordingData->udp_socket < 0) {
		return;
	}

	// Truncate if too long
	if (len >= sizeof(recordingData->udp_buffer)) {
		len = sizeof(recordingData->udp_buffer) - 1;
	}
	
	memcpy(recordingData->udp_buffer, data, len);
	recordingData->udp_buffer[len] = '\0';
	
	sendto(recordingData->udp_socket, recordingData->udp_buffer, len, MSG_NOSIGNAL,
		   (struct sockaddr*)&recordingData->udp_target_addr, sizeof(recordingData->udp_target_addr));
}

static void udp_stream_cleanup(SurviveRecordingData *recordingData) {
	if (recordingData && recordingData->udp_socket >= 0) {
		close(recordingData->udp_socket);
		recordingData->udp_socket = -1;
	}
}
void survive_recording_disconnect_process(struct SurviveObject *so) {
	SurviveRecordingData *recordingData = so->ctx ? so->ctx->recptr : 0;
	survive_recording_write_to_output(recordingData, "%s DISCONNECT\r\n", so->codename);
}

void survive_recording_config_process(SurviveObject *so, char *ct0conf, int len) {
	SurviveRecordingData *recordingData = so->ctx ? so->ctx->recptr : 0;
	if (recordingData == 0 || len < 0)
		return;

	char *buffer = SV_CALLOC(len + 1);
	memcpy(buffer, ct0conf, len);
	for (int i = 0; i < len; i++)
		if (buffer[i] == '\n' || buffer[i] == '\r')
			buffer[i] = ' ';

	survive_recording_write_to_output(recordingData, "%s CONFIG ", so->codename);
	write_to_output_raw(recordingData, buffer, len);

	write_to_output_raw(recordingData, "\r\n", 2);

	free(buffer);
}

void survive_recording_serial_number_process(SurviveObject *so) {
	SurviveRecordingData *recordingData = so->ctx ? so->ctx->recptr : 0;
	if (recordingData == 0)
		return;

	if (so->serial_number[0] != '\0') {
		survive_recording_write_to_output(recordingData, "%s SERIAL_NUMBER %s\r\n", so->codename, so->serial_number);
	}
}

void survive_recording_lighthouse_process(SurviveContext *ctx, uint8_t lighthouse, const SurvivePose *lh_pose) {
	SurviveRecordingData *recordingData = ctx->recptr;
	if (recordingData == 0)
		return;

	int8_t mode = ctx->bsd[lighthouse].mode;
	survive_recording_write_to_output(
		recordingData,
		"%d LH_POSE " FLT_PRINTF FLT_PRINTF FLT_PRINTF FLT_PRINTF FLT_PRINTF FLT_PRINTF FLT_PRINTF " %u\r\n", mode,
		lh_pose->Pos[0], lh_pose->Pos[1], lh_pose->Pos[2], lh_pose->Rot[0], lh_pose->Rot[1], lh_pose->Rot[2],
		lh_pose->Rot[3], ctx->bsd[lighthouse].BaseStationID);
}
void survive_recording_velocity_process(SurviveObject *so, uint8_t lighthouse, const SurviveVelocity *pose) {
	SurviveRecordingData *recordingData = so->ctx->recptr;
	if (recordingData == 0)
		return;

	survive_recording_write_to_output(
		recordingData, "%s VELOCITY " FLT_PRINTF FLT_PRINTF FLT_PRINTF FLT_PRINTF FLT_PRINTF FLT_PRINTF "\r\n",
		so->codename, pose->Pos[0], pose->Pos[1], pose->Pos[2], pose->AxisAngleRot[0], pose->AxisAngleRot[1],
		pose->AxisAngleRot[2]);
}
void survive_recording_raw_pose_process(SurviveObject *so, uint8_t lighthouse, const SurvivePose *pose) {
	SurviveRecordingData *recordingData = so->ctx->recptr;
	if (recordingData == 0)
		return;

	survive_recording_write_to_output(
		recordingData, "%s POSE " FLT_PRINTF FLT_PRINTF FLT_PRINTF FLT_PRINTF FLT_PRINTF FLT_PRINTF FLT_PRINTF "\r\n",
		so->codename, pose->Pos[0], pose->Pos[1], pose->Pos[2], pose->Rot[0], pose->Rot[1], pose->Rot[2], pose->Rot[3]);
}

void survive_recording_external_velocity_process(SurviveContext *ctx, const char *name, const SurviveVelocity *pose) {
	SurviveRecordingData *recordingData = ctx->recptr;
	if (recordingData == 0)
		return;

	survive_recording_write_to_output(
		recordingData, "%s EXTERNAL_VELOCITY " FLT_PRINTF FLT_PRINTF FLT_PRINTF FLT_PRINTF FLT_PRINTF FLT_PRINTF "\r\n",
		name, pose->Pos[0], pose->Pos[1], pose->Pos[2], pose->AxisAngleRot[0], pose->AxisAngleRot[1],
		pose->AxisAngleRot[2]);
}

void survive_recording_external_pose_process(SurviveContext *ctx, const char *name, const SurvivePose *pose) {
	SurviveRecordingData *recordingData = ctx->recptr;
	if (recordingData == 0)
		return;

	survive_recording_write_to_output(
		recordingData,
		"%s EXTERNAL_POSE " FLT_PRINTF FLT_PRINTF FLT_PRINTF FLT_PRINTF FLT_PRINTF FLT_PRINTF FLT_PRINTF "\n", name,
		pose->Pos[0], pose->Pos[1], pose->Pos[2], pose->Rot[0], pose->Rot[1], pose->Rot[2], pose->Rot[3]);
}

void survive_recording_info_process(SurviveContext *ctx, const char *fault) {
	SurviveRecordingData *recordingData = ctx->recptr;
	if (recordingData == 0)
		return;

	survive_recording_write_to_output(recordingData, "INFO LOG %s\r\n", fault);
}

void survive_recording_sync_process(SurviveObject *so, survive_channel channel, survive_timecode timecode, bool ootx,
									bool gen) {
	SurviveRecordingData *recordingData = so->ctx->recptr;

	const char* dev = so->codename;
	if (!recordingData || !recordingData->writeAngle) {
		return;
	}

	survive_recording_write_to_output(recordingData, SYNC_PRINTF, SYNC_PRINTF_ARGS);
}

void survive_recording_sweep_angle_process(SurviveObject *so, survive_channel channel, int sensor_id,
										   survive_timecode timecode, int8_t plane, FLT angle) {
	SurviveRecordingData *recordingData = so->ctx->recptr;
	if (!recordingData || !recordingData->writeAngle) {
		return;
	}

	const char *dev = so->codename;
	survive_recording_write_to_output(recordingData, SWEEP_ANGLE_PRINTF, SWEEP_ANGLE_PRINTF_ARGS);
}

void survive_recording_sweep_process(SurviveObject *so, survive_channel channel, int sensor_id,
									 survive_timecode timecode, bool flag) {
	SurviveRecordingData *recordingData = so->ctx->recptr;

	if (!recordingData || !recordingData->writeAngle) {
		return;
	}

	if (!recordingData->writeAngle)
		return;

	const char *dev = so->codename;
	survive_recording_write_to_output(recordingData, SWEEP_PRINTF, SWEEP_PRINTF_ARGS);
}

void survive_recording_button_process(SurviveObject *so, enum SurviveInputEvent eventType, enum SurviveButton buttonId,
									  const enum SurviveAxis *axisId, const SurviveAxisVal_t *axisVals) {
	SurviveRecordingData *recordingData = so->ctx->recptr;

	if (!recordingData) {
		return;
	}

	const char *dev = so->codename;
	survive_recording_write_to_output(recordingData, "%s BUTTON %u %u\r\n", dev, eventType, buttonId);
}
void survive_recording_angle_process(struct SurviveObject *so, int sensor_id, int acode, uint32_t timecode, FLT length,
									 FLT angle, uint32_t lh) {
	SurviveRecordingData *recordingData = so->ctx->recptr;
	if (!recordingData || !recordingData->writeAngle) {
		return;
	}

	survive_recording_write_to_output(recordingData, "%s A %d %d %u " FLT_PRINTF FLT_PRINTF "%u\r\n", so->codename,
									  sensor_id, acode, timecode, length, angle, lh);
}

void survive_recording_lightcap(SurviveObject *so, LightcapElement *le) {
	SurviveRecordingData *recordingData = so->ctx->recptr;
	if (recordingData == 0)
		return;

	if (recordingData->writeRawLight) {
		survive_recording_write_to_output(recordingData, "%s C %d %u %u\r\n", so->codename, le->sensor_id,
										  le->timestamp, le->length);
	}
}

void survive_recording_light_process(struct SurviveObject *so, int sensor_id, int acode, int timeinsweep,
									 uint32_t timecode, uint32_t length, uint32_t lh) {
	SurviveRecordingData *recordingData = so->ctx->recptr;
	if (recordingData == 0)
		return;

	if (!recordingData->writeAngle) {
	  return;
	}
	
	if (acode == -1) {
		survive_recording_write_to_output(recordingData, "%s S %d %d %d %u %u %u\r\n", so->codename, sensor_id, acode,
										  timeinsweep, timecode, length, lh);
		return;
	}

	const char *LH_ID = 0;
	const char *LH_Axis = 0;

	switch (acode) {
	case 0:
	case 2:
		LH_ID = "L";
		LH_Axis = "X";
		break;
	case 1:
	case 3:
		LH_ID = "L";
		LH_Axis = "Y";
		break;
	case 4:
	case 6:
		LH_ID = "R";
		LH_Axis = "X";
		break;
	case 5:
	case 7:
		LH_ID = "R";
		LH_Axis = "Y";
		break;
	}

	survive_recording_write_to_output(recordingData, "%s %s %s %d %d %d %u %u %u\r\n", so->codename, LH_ID, LH_Axis,
									  sensor_id, acode, timeinsweep, timecode, length, lh);
}

void survive_recording_imu_scales(struct SurviveObject *so, int gyro_scale_mode, int acc_scale_mode) {
    SurviveRecordingData *recordingData = so->ctx->recptr;
    if (recordingData == 0)
        return;

    survive_recording_write_to_output(recordingData,
                                      "%s IMU_SCALES %d %d\r\n",
                                      so->codename, gyro_scale_mode, acc_scale_mode);

}
void survive_recording_imu_process(struct SurviveObject *so, int mask, const FLT *accelgyro, uint32_t timecode,
								   int id) {
	SurviveRecordingData *recordingData = so->ctx->recptr;
	if (recordingData == 0)
		return;

	if (!recordingData->writeCalIMU) {
		return;
	}

	survive_recording_write_to_output(recordingData,
									  "%s I %d %u " FLT_PRINTF FLT_PRINTF FLT_PRINTF FLT_PRINTF FLT_PRINTF FLT_PRINTF
									  " " FLT_PRINTF FLT_PRINTF FLT_PRINTF "%d\r\n",
									  so->codename, mask, timecode, accelgyro[0], accelgyro[1], accelgyro[2],
									  accelgyro[3], accelgyro[4], accelgyro[5], accelgyro[6], accelgyro[7],
									  accelgyro[8], id);
}

void survive_recording_raw_imu_process(struct SurviveObject *so, int mask, const FLT *accelgyro, uint32_t timecode,
									   int id) {
	SurviveRecordingData *recordingData = so->ctx->recptr;
	if (recordingData == 0)
		return;

	if (!recordingData->writeIMU) {
		return;
	}

	survive_recording_write_to_output(recordingData,
									  "%s i %d %u " FLT_PRINTF FLT_PRINTF FLT_PRINTF FLT_PRINTF FLT_PRINTF FLT_PRINTF
									  " " FLT_PRINTF FLT_PRINTF FLT_PRINTF "%d\r\n",
									  so->codename, mask, timecode, accelgyro[0], accelgyro[1], accelgyro[2],
									  accelgyro[3], accelgyro[4], accelgyro[5], accelgyro[6], accelgyro[7],
									  accelgyro[8], id);
}

void survive_destroy_recording(SurviveContext *ctx) {
	if (ctx->recptr) {
		SurviveRecordingData_detach_config(ctx, ctx->recptr);
		if (ctx->recptr->output_file) {
			gzclose(ctx->recptr->output_file);
		}
		udp_stream_cleanup(ctx->recptr);
		free(ctx->recptr);
		ctx->recptr = 0;
	}
}

void survive_record_config(SurviveContext *ctx, const char *tag, uint8_t type, const char *desc, const char *def_value,
						   void *user) {
	char buf[128];
	survive_config_as_str(ctx, buf, sizeof(buf), tag, "");
	survive_recording_write_to_output(ctx->recptr, "OPTION %s %c %s\n", tag, type, buf);
}

void survive_install_recording(SurviveContext *ctx) {
	const char *dataout_file = survive_configs(ctx, "record", SC_GET, "");
	int record_to_stdout = survive_configi(ctx, "record-stdout", SC_GET, 0);
	int udp_stream_enabled = survive_configi(ctx, "udp-stream", SC_GET, 0);

	if (strlen(dataout_file) > 0 || record_to_stdout || udp_stream_enabled) {
		ctx->recptr = SV_CALLOC(sizeof(struct SurviveRecordingData));
		ctx->recptr->ctx = ctx;
		SurviveRecordingData_attach_config(ctx, ctx->recptr);
		
		// Initialize UDP streaming
		ctx->recptr->udpStreamEnabled = udp_stream_enabled;
		ctx->recptr->udp_socket = -1;
		if (udp_stream_enabled) {
			udp_stream_init(ctx->recptr);
		}
		
		if (strlen(dataout_file) > 0) {
			if (strstr(dataout_file, ".pcap")) {
				int (*usb_driver)(SurviveContext *) = (int (*)(SurviveContext *))GetDriver("DriverRegUSBMon_Record");
				if (usb_driver) {
					usb_driver(ctx);
					return;
				}
				SV_WARN("Playback file %s is a USB packet capture, but the usbmon playback driver does not exist.",
						dataout_file);
				return;
			} else {

				bool useCompression = strncmp(dataout_file + strlen(dataout_file) - 3, ".gz", 3) == 0;

				ctx->recptr->output_file = gzopen(dataout_file, useCompression ? "w6F" : "wT");
				if (ctx->recptr->output_file == 0) {
					SV_INFO("Could not open %s for writing", dataout_file);
					free(ctx->recptr);
					ctx->recptr = 0;
					return;
				}
				SV_INFO("Recording to '%s' Compression: %d", dataout_file, useCompression);
			}
		}

		ctx->recptr->alwaysWriteStdOut = record_to_stdout;
		if (record_to_stdout) {
			SV_INFO("Recording to stdout");
		}
	}

	survive_config_iterate(ctx, survive_record_config, ctx->recptr);
}
