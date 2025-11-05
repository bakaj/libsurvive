#include "generated/survive_reproject.aux.generated.h"
#include "math.h"
#include "survive_kalman_lighthouses.h"
#include "survive_kalman_tracker.h"
#include "survive_recording.h"
#include <assert.h>
#include <linmath.h>
#include <stdint.h>
#include <stdio.h>
#include <survive.h>

#include "generated/common_math.gen.h"

#define _USE_MATH_DEFINES // for C
#include <math.h>
#include <poser.h>
#include <stdlib.h>
#include <string.h>

void survive_poseAA2pose_jacobian(struct CnMat *G, const LinmathAxisAnglePose *poseAA) {
	CN_CREATE_STACK_MAT(Gp, 4, 3);

	for (int i = 0; i < 3; i++)
		cnMatrixSet(G, i, i, 1);

	gen_axisangle2quat_jac_axis_angle(Gp.data, poseAA->AxisAngleRot);
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 3; j++) {
			cnMatrixSet(G, i + 3, j + 3, cnMatrixGet(&Gp, i, j));
		}
	}
}

void survive_pose2poseAA_jacobian(struct CnMat *G, const LinmathPose *pose) {
	CN_CREATE_STACK_MAT(Gp, 3, 4);

	for (int i = 0; i < 3; i++)
		cnMatrixSet(G, i, i, 1);

	gen_quat2axisangle_jac_q(Gp.data, pose->Rot);
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 4; j++) {
			cnMatrixSet(G, i + 3, j + 3, cnMatrixGet(&Gp, i, j));
		}
	}
	assert(cn_is_finite(G));
}
void survive_covariance_pose2poseAA(struct CnMat *R_aa, const LinmathPose *pose, const struct CnMat *R_q) {
	CN_CREATE_STACK_MAT(G, R_aa->rows, R_q->rows);
	survive_pose2poseAA_jacobian(&G, pose);
	if(R_aa->cols == R_aa->rows) {
		gemm_ABAt_add_scaled(R_aa, &G, R_q, 0, 1, 1, 0);
	} else {
		CN_CREATE_STACK_MAT(R_aa_full, 6, 6);
		CN_CREATE_STACK_MAT(R_q_full, 7, 7);
		cn_set_diag(&R_q_full, R_q->data);
		gemm_ABAt_add_scaled(&R_aa_full, &G, &R_q_full, 0, 1, 1, 0);
		cn_get_diag(&R_aa_full, cn_as_vector(R_aa), R_aa->rows);
	}
}
void survive_covariance_poseAA2pose(struct CnMat *R_q, const LinmathAxisAnglePose *poseAA, const struct CnMat *R_aa) {
	CN_CREATE_STACK_MAT(G, R_q->rows, R_aa->rows);
	survive_poseAA2pose_jacobian(&G, poseAA);
	gemm_ABAt_add_scaled(R_q, &G, R_aa, 0, 1, 1, 0);
}

// Calculate tracker-fixed coordinate frame from sensor positions using PCA
// Takes sensor positions in trackref space (original tracker geometry frame)
// This ensures the coordinate system is based on physical tracker geometry, not IMU orientation
// This function can be called during device loading when positions are still in trackref space (avoiding double transform)
static void calculate_tracker_fixed_frame_from_positions(const LinmathPoint3d *sensor_positions, size_t sensor_count, SurvivePose *arb2tracker_fixed) {
	if (sensor_count < 3) {
		// Not enough sensors to define a coordinate frame
		*arb2tracker_fixed = (SurvivePose){.Rot = {1.}};
		return;
	}

	// 1. Calculate centroid (tracker center) in trackref space
	LinmathPoint3d centroid = {0};
	for (size_t i = 0; i < sensor_count; i++) {
		add3d(centroid, centroid, sensor_positions[i]);
	}
	scale3d(centroid, centroid, 1.0 / sensor_count);

	// 2. Center sensor positions and compute covariance matrix
	CN_CREATE_STACK_MAT(cov, 3, 3);
	CN_CREATE_STACK_MAT(centered_data, sensor_count, 3);
	cnSetZero(&cov);
	cnSetZero(&centered_data);

	// Center the data (using trackref-space positions)
	for (size_t i = 0; i < sensor_count; i++) {
		LinmathPoint3d centered;
		sub3d(centered, sensor_positions[i], centroid);
		for (int j = 0; j < 3; j++) {
			cnMatrixSet(&centered_data, i, j, centered[j]);
		}
	}

	// Compute covariance matrix: cov = (1/n) * centered_data^T * centered_data
	CN_CREATE_STACK_MAT(centered_data_t, 3, sensor_count);
	cnTranspose(&centered_data, &centered_data_t);
	cnGEMM(&centered_data_t, &centered_data, 1.0 / sensor_count, 0, 0, &cov, 0);

	// 3. Perform SVD to get principal components (eigenvectors)
	CN_CREATE_STACK_MAT(S, 3, 1);      // Singular values
	CN_CREATE_STACK_MAT(U, 3, 3);      // Left singular vectors (principal components)
	CN_CREATE_STACK_MAT(Vt, 3, 3);     // Right singular vectors (transpose)

	cnSVD(&cov, &S, &U, &Vt, CN_SVD_MODIFY_A);

	// Principal axes are columns of U (for covariance matrix, U contains eigenvectors)
	// Extract rotation matrix from U
	FLT rot_matrix[3][3];
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			rot_matrix[i][j] = cnMatrixGet(&U, i, j);
		}
	}

	// Ensure right-handed coordinate system
	FLT det = rot_matrix[0][0] * (rot_matrix[1][1] * rot_matrix[2][2] - rot_matrix[1][2] * rot_matrix[2][1]) -
			  rot_matrix[0][1] * (rot_matrix[1][0] * rot_matrix[2][2] - rot_matrix[1][2] * rot_matrix[2][0]) +
			  rot_matrix[0][2] * (rot_matrix[1][0] * rot_matrix[2][1] - rot_matrix[1][1] * rot_matrix[2][0]);

	if (det < 0) {
		// Flip last column to ensure right-handed system
		for (int i = 0; i < 3; i++) {
			rot_matrix[i][2] = -rot_matrix[i][2];
		}
	}

	// 4. Convert rotation matrix to quaternion
	quatfrommatrix33(arb2tracker_fixed->Rot, rot_matrix[0]);

	// 5. Set translation to move to tracker center (origin)
	scale3d(arb2tracker_fixed->Pos, centroid, -1.0);
}

// Wrapper that works with SurviveObject - transforms from IMU space to trackref space if needed
// Made non-static so it can be called from survive_process.c for regular pose updates
SURVIVE_EXPORT void calculate_tracker_fixed_frame(SurviveObject *so, SurvivePose *arb2tracker_fixed) {
	if (!so->has_sensor_locations || so->sensor_ct < 3) {
		*arb2tracker_fixed = (SurvivePose){.Rot = {1.}};
		return;
	}

	// Transform sensor positions from IMU space back to trackref space
	LinmathPoint3d *sensor_positions_trackref = (LinmathPoint3d *)alloca(sizeof(LinmathPoint3d) * so->sensor_ct);
	for (int i = 0; i < so->sensor_ct; i++) {
		ApplyPoseToPoint(&sensor_positions_trackref[i], &so->imu2trackref, &so->sensor_locations[i * 3]);
	}

	// Calculate using trackref-space positions
	calculate_tracker_fixed_frame_from_positions(sensor_positions_trackref, so->sensor_ct, arb2tracker_fixed);
}

// Transform lighthouse pose from object space (arbitrary frame) to tracker-fixed space
// Made non-static so it can be called from survive_process.c
SURVIVE_EXPORT void transform_to_tracker_fixed(const SurvivePose *lh2object, 
									   const SurvivePose *arb2tracker_fixed,
									   SurvivePose *lh2tracker_fixed) {
	// Transform: lh2tracker_fixed = arb2tracker_fixed * lh2object
	ApplyPoseToPose(lh2tracker_fixed, arb2tracker_fixed, lh2object);
}

SURVIVE_EXPORT int32_t PoserData_size(const PoserData *poser_data) {
	switch (poser_data->pt) {
	case POSERDATA_DISASSOCIATE:
		return sizeof(PoserData);
	case POSERDATA_IMU:
		return sizeof(PoserDataIMU);
	case POSERDATA_LIGHT:
	case POSERDATA_SYNC:
		return sizeof(PoserDataLightGen1);
	case POSERDATA_LIGHT_GEN2:
	case POSERDATA_SYNC_GEN2:
		return sizeof(PoserDataLightGen2);
	default: break;
	}
	assert(false);
	return 0;
}

STATIC_CONFIG_ITEM(CENTER_ON_LH0, "center-on-lh0", 'b',
				   "Alternative scheme for setting initial position; LH0 is 0, 0 looking in the +X direction", 0)

STATIC_CONFIG_ITEM(HAPTIC_ON_CALIBRATE, "haptic-on-calibrate", 'b',
				   "Trigger a haptic pulse when lighthouse positions are solved", 0);

STATIC_CONFIG_ITEM(LIGHTHOUSE_NORMALIZE_ANGLE, "normalize-lighthouse-angle", 'f',
				   "Angle about Z to adust calibration by", 0.);

void PoserData_poser_pose_func(PoserData *poser_data, SurviveObject *so, const SurvivePose *imu2world, FLT error,
							   const struct CnMat *R) {
	SurviveContext *ctx = so->ctx;
	for (int i = 0; i < 3; i++) {
		assert(!isnan(imu2world->Pos[i]));
		if (fabs(imu2world->Pos[i]) > 20) {
			SV_WARN("Squelching reported pose of " SurvivePose_format " for %s; values are invalid",
					SURVIVE_POSE_EXPAND(*imu2world), so->codename);
			return;
		}
	}
	if (poser_data->poseproc) {
		poser_data->poseproc(so, poser_data->timecode, imu2world, poser_data->userdata);
	} else {
		//FLT p_e = error;
		//FLT r_e = error;
		//FLT R[7] = {p_e, p_e, p_e, r_e, r_e, r_e, r_e };
		CnMat Rt = cnMatConstView(7, 7, R, 0, 0);
		survive_kalman_tracker_integrate_observation(poser_data, so->tracker, imu2world, &Rt);
	}
}
void PoserData_poser_pose_func_with_velocity(PoserData *poser_data, SurviveObject *so, const SurvivePose *imu2world,
											 const SurviveVelocity *velocity) {
	SURVIVE_INVOKE_HOOK_SO(velocity, so, poser_data->timecode, velocity);
	PoserData_poser_pose_func(poser_data, so, imu2world, -1, 0);
}

void PoserData_lighthouse_pose_func(PoserData *poser_data, SurviveObject *so, uint8_t lighthouse,
									SurvivePose *lighthouse_pose, const CnMat * R, SurvivePose *object_pose) {
	if (poser_data && poser_data->lighthouseposeproc) {
		for (int i = 0; i < 7; i++)
			assert(!isnan(((FLT *)lighthouse_pose)[i]));

		assert(!quatiszero(lighthouse_pose->Rot));

		if (object_pose && quatiszero(object_pose->Rot)) {
			*object_pose = (SurvivePose){.Rot = {1.}};
		}

		poser_data->lighthouseposeproc(so, lighthouse, lighthouse_pose, object_pose, poser_data->userdata);
	} else {
		const FLT up[3] = {0, 0, 1};
		SurviveContext *ctx = so->ctx;
		SurvivePose origLHPose = *lighthouse_pose;
		if (quatmagnitude(lighthouse_pose->Rot) == 0) {
			SV_INFO("Pose func called with invalid pose.");
			return;
		}

		// Assume that the space solved for is valid but completely arbitrary. We are going to do a few things:
		// a) Using the gyro data, normalize it so that gravity is pushing straight down along Z
		// c) Assume the object is at origin
		// b) Place the first lighthouse on the X axis by rotating around Z
		//
		// This calibration setup has the benefit that as long as the user is calibrating on the same flat surface,
		// calibration results will be roughly identical between all posers no matter the orientation the object is
		// lying
		// in.
		//
		// We might want to go a step further and affix the first lighthouse in a given pose that preserves up so that
		// it doesn't matter where on that surface the object is.
		bool worldEstablished = quatmagnitude(object_pose->Rot) != 0;
		SurvivePose object2arb = {.Rot = {1.}};
		if (object_pose && !quatiszero(object_pose->Rot))
			object2arb = *object_pose;
		SurvivePose lighthouse2arb = *lighthouse_pose;

		SurvivePose obj2world, lighthouse2world;
		// Purposefully only set this once. It should only depend on the first (calculated) lighthouse
		if (!worldEstablished) {
			bool centerOnLh0 = survive_configi(so->ctx, "center-on-lh0", SC_GET, 0);

			// Start by just moving from whatever arbitrary space into object space.
			SurvivePose arb2object;
			InvertPose(&arb2object, &object2arb);

			SurvivePose lighthouse2obj;
			ApplyPoseToPose(&lighthouse2obj, &arb2object, &lighthouse2arb);

			if (so->ctx->recptr && so->ctx->recptr->writeLHObjectSpace) {
				survive_recording_lighthouse_object_space_normal(so, lighthouse, &lighthouse2obj);
			}

			// NEW ADDITIVE CODE: Record tracker-fixed poses if enabled
			if (so->ctx->recptr && so->ctx->recptr->writeLHTrackerFixed) {
				// Calculate tracker-fixed frame (cached per object pointer)
				static SurvivePose arb2tracker_fixed_cache = {0};
				static SurviveObject *cached_so = 0;
				static bool frame_calculated = false;
				
				if (cached_so != so || !frame_calculated) {
					calculate_tracker_fixed_frame(so, &arb2tracker_fixed_cache);
					cached_so = so;
					frame_calculated = true;
				}
				
				// Transform to tracker-fixed
				SurvivePose lh2tracker_fixed;
				transform_to_tracker_fixed(&lighthouse2obj, &arb2tracker_fixed_cache, &lh2tracker_fixed);
				
				// Record
				survive_recording_lighthouse_tracker_fixed_process(so, lighthouse, &lh2tracker_fixed);
			}

			SurvivePose arb2world = arb2object;

			// Find the poses that map to the above

			// Find the space with the same origin, but rotated so that gravity is up
			SurvivePose lighthouse2objUp = {0}, object2objUp = {0};
			FLT accel_mag = norm3d(so->activations.accel);
			if (accel_mag != 0.0 && !isnan(accel_mag)) {
				quatfrom2vectors(object2objUp.Rot, so->activations.accel, up);
			} else {
				SV_WARN("Calibration didn't have valid IMU data for %s; couldn't establish 'up' vector.", so->codename);
				object2objUp.Rot[0] = 1.0;
			}

			// Calculate the pose of the lighthouse in this space
			ApplyPoseToPose(&lighthouse2objUp, &object2objUp, &lighthouse2obj);
			ApplyPoseToPose(&arb2world, &object2objUp, &arb2world);

			// Find what angle we need to rotate about Z by to get to 90 degrees.
			FLT ang = atan2(lighthouse2objUp.Pos[1], lighthouse2objUp.Pos[0]);
			FLT ang_target = M_PI / 2.;
			FLT euler[3] = {0, 0, ang_target - ang};
			SurvivePose objUp2World = {0};
			quatfromeuler(objUp2World.Rot, euler);

			ApplyPoseToPose(&arb2world, &objUp2World, &arb2world);
			ApplyPoseToPose(&obj2world, &arb2world, &object2arb);
			ApplyPoseToPose(&lighthouse2world, &arb2world, &lighthouse2arb);

			if (centerOnLh0) {
				sub3d(obj2world.Pos, obj2world.Pos, lighthouse2world.Pos);
				lighthouse2world.Pos[0] = lighthouse2world.Pos[1] = lighthouse2world.Pos[2] = 0.0;

				LinmathPoint3d camFwd = {0, 0, -1}, worldFwd = {0};
				ApplyPoseToPoint(worldFwd, &lighthouse2world, camFwd);
				FLT ang = atan2(worldFwd[1], worldFwd[0]);
				FLT euler[3] = {0, 0, M_PI / 2 - ang};

				SurvivePose rotate2center = {0};
				quatfromeuler(rotate2center.Rot, euler);

				ApplyPoseToPose(&obj2world, &rotate2center, &obj2world);
				ApplyPoseToPose(&lighthouse2world, &rotate2center, &lighthouse2world);
			}

			*object_pose = obj2world;
		} else {
			lighthouse2world = *lighthouse_pose;
			obj2world = *object_pose;
		}

		for (int i = 0; i < 7; i++)
			assert(!isnan(((FLT *)&lighthouse2world)[i]));

		survive_kalman_lighthouse_integrate_observation(so->ctx->bsd[lighthouse].tracker, lighthouse_pose, R);
	}
}

int8_t survive_get_reference_bsd(SurviveContext *ctx, SurvivePose *lighthouse_pose, uint32_t lighthouse_count) {
	uint32_t reference_basestation = survive_configi(ctx, "reference-basestation", SC_GET, 0);
	int8_t ref = -1;
	for (int lh = 0; lh < lighthouse_count; lh++) {
		SurvivePose lh2object = lighthouse_pose[lh];
		if (quatmagnitude(lh2object.Rot) != 0.0) {
			uint32_t lh0 = ref != -1 ? ref : 0;
			bool hasBetterId = reference_basestation == 0 ? (ctx->bsd[lh].BaseStationID <= ctx->bsd[lh0].BaseStationID)
														  : reference_basestation == ctx->bsd[lh].BaseStationID;
			bool preferThisBSD = ref == -1 || hasBetterId;
			if (preferThisBSD) {
				ref = lh;
			}
		}
	}
	return ref;
}

int8_t survive_get_ctx_reference_bsd(SurviveContext *ctx) {
	SurvivePose cameras[NUM_GEN2_LIGHTHOUSES] =  { 0 };
	for(int i = 0;i < ctx->activeLighthouses;i++) {
		if(ctx->bsd[i].PositionSet)
			cameras[i] = *survive_get_lighthouse_position(ctx, i);
	}
	return survive_get_reference_bsd(ctx, cameras, ctx->activeLighthouses);
}

void PoserData_normalize_scene(SurviveContext *ctx, SurvivePose *lighthouse_pose, uint32_t lighthouse_count,
							   SurvivePose *object_pose, CnMat* R) {
	FLT lhNormAngleOffset = survive_configf(ctx, LIGHTHOUSE_NORMALIZE_ANGLE_TAG, SC_GET, 0);
	uint32_t lh_indices[NUM_GEN2_LIGHTHOUSES] = {0};
	uint32_t cnt = 0;

	uint32_t reference_basestation = survive_configi(ctx, "reference-basestation", SC_GET, 0);
	SurvivePose object2arb = *object_pose;

	for (int lh = 0; lh < lighthouse_count; lh++) {
		SurvivePose lh2object = lighthouse_pose[lh];
		if (quatmagnitude(lh2object.Rot) != 0.0) {
			lh_indices[cnt] = lh;
			uint32_t lh0 = lh_indices[0];
			bool preferThisBSD = reference_basestation == 0 ? (ctx->bsd[lh].BaseStationID < ctx->bsd[lh0].BaseStationID)
															: reference_basestation == ctx->bsd[lh].BaseStationID;
			if (preferThisBSD) {
				lh_indices[0] = lh;
				lh_indices[cnt] = lh0;
			}
			cnt++;
		}
	}

	SurvivePose *preferredLH = &lighthouse_pose[lh_indices[0]];
	FLT ang = atan2(preferredLH->Pos[1], preferredLH->Pos[0]);
	FLT ang_target = M_PI / 2. + lhNormAngleOffset * M_PI / 180.;
	FLT euler[3] = {0, 0, ang_target - ang};
	SurvivePose arb2world = {0};
	quatfromeuler(arb2world.Rot, euler);

	bool centerOnLh0 = survive_configi(ctx, "center-on-lh0", SC_GET, 0);
	if(centerOnLh0) {
		SurvivePose offset = { .Rot = {1} };
		scalend(offset.Pos, preferredLH->Pos, -1, 3);
		ApplyPoseToPose(&arb2world, &arb2world, &offset);
	}

	ApplyPoseToPose(object_pose, &arb2world, &object2arb);
	int lh_idx = 0;
	for (int lh = 0; lh < lighthouse_count; lh++) {
		SurvivePose *lh2object = &lighthouse_pose[lh];
		if (quatmagnitude(lh2object->Rot) != 0.0) {
			ApplyPoseToPose(lh2object, &arb2world, lh2object);

			if(R) {
				CnMat LH_R = cnMatConstView(7, 7, R, lh_idx * 7, lh_idx * 7);
				CN_CREATE_STACK_MAT(jac, 7, 7);
				apply_pose_to_pose_jac_rhs(&jac, &arb2world, lh2object);
				cn_ABAt_add(&LH_R, &jac, &LH_R, 0);
			}
			lh_idx++;
		}
	}
}

void PoserData_lighthouse_poses_func(PoserData *poser_data, SurviveObject *so, SurvivePose *lighthouse_pose,
									 const struct CnMat *R, uint32_t lighthouse_count, SurvivePose *object_pose) {

	if (poser_data && poser_data->lighthouseposeproc) {
		for (int lighthouse = 0; lighthouse < lighthouse_count; lighthouse++) {
			if (quatiszero(lighthouse_pose[lighthouse].Rot))
				continue;

			if (object_pose && quatiszero(object_pose->Rot)) {
				*object_pose = (SurvivePose){.Rot = {1.}};
			}
			
			// Record lighthouse pose in object space for every frame/update
			// lighthouse_pose[lighthouse] is already in object space when passed to custom callback
			if (so->ctx->recptr && so->ctx->recptr->writeLHObjectSpace) {
				SurvivePose lh2object = lighthouse_pose[lighthouse];
				quatnormalize(lh2object.Rot, lh2object.Rot);
				survive_recording_lighthouse_object_space_normal(so, lighthouse, &lh2object);
			}

			// NEW ADDITIVE CODE: Record tracker-fixed poses if enabled
			if (so->ctx->recptr && so->ctx->recptr->writeLHTrackerFixed) {
				SurvivePose lh2object = lighthouse_pose[lighthouse];
				quatnormalize(lh2object.Rot, lh2object.Rot);
				
				// Calculate tracker-fixed frame (cached per object pointer)
				static SurvivePose arb2tracker_fixed_cache = {0};
				static SurviveObject *cached_so = 0;
				static bool frame_calculated = false;
				
				if (cached_so != so || !frame_calculated) {
					calculate_tracker_fixed_frame(so, &arb2tracker_fixed_cache);
					cached_so = so;
					frame_calculated = true;
				}
				
				// Transform to tracker-fixed
				SurvivePose lh2tracker_fixed;
				transform_to_tracker_fixed(&lh2object, &arb2tracker_fixed_cache, &lh2tracker_fixed);
				
				// Record
				survive_recording_lighthouse_tracker_fixed_process(so, lighthouse, &lh2tracker_fixed);
			}
			
			poser_data->lighthouseposeproc(so, lighthouse, &lighthouse_pose[lighthouse], object_pose,
										   poser_data->userdata);
		}
	} else {
		bool hapticOnCalibrate = survive_configi(so->ctx, HAPTIC_ON_CALIBRATE_TAG, SC_GET, 0);

		SurvivePose object2World;
		if (object_pose == 0 || quatiszero(object_pose->Rot))
			object2World = so->OutPoseIMU;
		else
			object2World = *object_pose;

		bool worldEstablished = !quatiszero(object2World.Rot);

		uint32_t lh_indices[NUM_GEN2_LIGHTHOUSES] = {0};
		uint32_t cnt = 0;

		uint32_t reference_basestation = survive_configi(so->ctx, "reference-basestation", SC_GET, 0);

		for (int lh = 0; lh < lighthouse_count; lh++) {
			SurvivePose lh2object = lighthouse_pose[lh];
			if (quatmagnitude(lh2object.Rot) != 0.0) {
				lh_indices[cnt] = lh;
				uint32_t lh0 = lh_indices[0];
				bool preferThisBSD = reference_basestation == 0
										 ? (so->ctx->bsd[lh].BaseStationID < so->ctx->bsd[lh0].BaseStationID)
										 : reference_basestation == so->ctx->bsd[lh].BaseStationID;
				if (preferThisBSD) {
					lh_indices[0] = lh;
					lh_indices[cnt] = lh0;
				}
				cnt++;
			}
		}

		struct SurviveContext *ctx = so->ctx;
		SV_INFO("Using LH %d (%08x) as reference lighthouse", lh_indices[0], so->ctx->bsd[lh_indices[0]].BaseStationID);
		for (int lh_idx = 0; lh_idx < cnt; lh_idx++) {
			int lh = lh_indices[lh_idx];

			SurvivePose lh2object = lighthouse_pose[lh];
			quatnormalize(lh2object.Rot, lh2object.Rot);

			// Record lighthouse pose in object space for every frame/update
			if (so->ctx->recptr && so->ctx->recptr->writeLHObjectSpace) {
				survive_recording_lighthouse_object_space_normal(so, lh, &lh2object);
			}

			// NEW ADDITIVE CODE: Record tracker-fixed poses if enabled
			if (so->ctx->recptr && so->ctx->recptr->writeLHTrackerFixed) {
				// Calculate tracker-fixed frame (cached per object pointer)
				static SurvivePose arb2tracker_fixed_cache = {0};
				static SurviveObject *cached_so = 0;
				static bool frame_calculated = false;
				
				if (cached_so != so || !frame_calculated) {
					calculate_tracker_fixed_frame(so, &arb2tracker_fixed_cache);
					cached_so = so;
					frame_calculated = true;
				}
				
				// Transform to tracker-fixed
				SurvivePose lh2tracker_fixed;
				transform_to_tracker_fixed(&lh2object, &arb2tracker_fixed_cache, &lh2tracker_fixed);
				
				// Record
				survive_recording_lighthouse_tracker_fixed_process(so, lh, &lh2tracker_fixed);
			}

			SurvivePose lh2world = lh2object;
			CnMat LH_R;
			if(R)
				LH_R = cnMatConstView(7, 7, R, lh_idx * 7, lh_idx * 7);
			if (!quatiszero(object2World.Rot) && worldEstablished == false) {
				ApplyPoseToPose(&lh2world, &object2World, &lh2object);
			}

			PoserData_lighthouse_pose_func(poser_data, so, lh, &lh2world, R ? &LH_R : 0,
										   &object2World);
		}

		if (hapticOnCalibrate) {
			for (int i = 0; i < ctx->objs_ct; i++) {
				survive_haptic(ctx->objs[i], 120, 1., .05);
			}
		}

		if (object_pose)
			*object_pose = object2World;
	}
}

FLT survive_lighthouse_adjust_confidence(SurviveContext *ctx, uint8_t bsd_idx, FLT v) {
	ctx->bsd[bsd_idx].confidence += v;

	if (ctx->bsd[bsd_idx].confidence < 0) {
		ctx->bsd[bsd_idx].PositionSet = 0;
		SV_WARN("Position for LH%d seems bad; queuing for recal", bsd_idx);
	} else if (ctx->bsd[bsd_idx].confidence > 1.) {
		return ctx->bsd[bsd_idx].confidence = 1;
	}

	return ctx->bsd[bsd_idx].confidence;
}

SURVIVE_EXPORT FLT survive_adjust_confidence(SurviveObject *so, FLT delta) {
	so->poseConfidence += delta;
	if (so->poseConfidence < 0) {
	}
	return so->poseConfidence;
}

void survive_poser_invoke(SurviveObject *so, PoserData *poserData, size_t poserDataSize) {
	if (so->ctx->PoserFn) {
		so->ctx->PoserFn(so, poserData);
	}
}

struct survive_threaded_poser {
	og_thread_t thread;

	union PoserDataAll PoserData;
	bool active, has_new_data;
	og_cv_t data_available;
	og_mutex_t data_available_lock;

	SurviveObject *so;
	PoserCB innerPoser;

	uint32_t run_count, new_data_count;
};

void *survive_threaded_poser_thread_fn(void *_poser) {
	struct survive_threaded_poser *self = (struct survive_threaded_poser *)_poser;
	OGLockMutex(self->data_available_lock);
	while (self->active) {
		OGWaitCond(self->data_available, self->data_available_lock);

		while (self->has_new_data) {
			SurviveObject *so = self->so;
			self->has_new_data = false;
			OGUnlockMutex(self->data_available_lock);

			survive_get_ctx_lock(self->so->ctx);
			self->innerPoser(so, &self->PoserData.pd);
			survive_release_ctx_lock(self->so->ctx);
			self->run_count++;

			OGLockMutex(self->data_available_lock);
		}
	}
	OGUnlockMutex(self->data_available_lock);
	return 0;
}

struct survive_threaded_poser *survive_create_threaded_poser(SurviveObject *so, PoserCB innerPoser) {
	struct survive_threaded_poser *poser = SV_CALLOC(sizeof(struct survive_threaded_poser));
	poser->so = so;
	poser->innerPoser = innerPoser;
	poser->data_available = OGCreateConditionVariable();
	poser->data_available_lock = OGCreateMutex();
	poser->active = 1;
	SurviveContext *ctx = so->ctx;
	SV_VERBOSE(10, "Creating threaded poser for %s", survive_colorize(so->codename));
	poser->thread = OGCreateThread(survive_threaded_poser_thread_fn, "threaded poser", poser);
	return poser;
}
int survive_threaded_poser_fn(SurviveObject *so, PoserData *pd) {
	void **user = survive_object_plugin_data(so, survive_threaded_poser_fn);
	struct survive_threaded_poser *self = (struct survive_threaded_poser *)*user;
	assert(self);

	switch (pd->pt) {
	case POSERDATA_DISASSOCIATE: {
		OGLockMutex(self->data_available_lock);
		self->active = 0;
		OGSignalCond(self->data_available);
		OGUnlockMutex(self->data_available_lock);
		survive_release_ctx_lock(self->so->ctx);
		OGJoinThread(self->thread);
		survive_get_ctx_lock(self->so->ctx);

		self->innerPoser(so, pd);

		SurviveContext *ctx = so->ctx;
		SV_VERBOSE(5, "Threaded stats:");
		SV_VERBOSE(5, "\tRan       %d", self->run_count);
		SV_VERBOSE(5, "\tNew data  %d", self->new_data_count);

		OGDeleteMutex(self->data_available_lock);
		OGDeleteConditionVariable(self->data_available);
		free(self);
		*user = 0;
		return 0;
	}
	case POSERDATA_SYNC_GEN2:
	case POSERDATA_SYNC: {
		OGLockMutex(self->data_available_lock);
		memcpy(&self->PoserData.pd, pd, PoserData_size(pd));
		self->has_new_data = true;
		self->new_data_count++;
		OGSignalCond(self->data_available);
		OGUnlockMutex(self->data_available_lock);
		return 0;
	}
	default: {
		if (self->innerPoser) {
			self->innerPoser(so, pd);
		}
	}
	}

	return 0;
}

SURVIVE_EXPORT int PoserDataLight_axis(const struct PoserDataLight *pdl) {
	switch (pdl->hdr.pt) {
	case POSERDATA_LIGHT:
	case POSERDATA_SYNC:
		return (((PoserDataLightGen1 *)pdl)->acode & 1);
		break;
	case POSERDATA_SYNC_GEN2:
	case POSERDATA_LIGHT_GEN2:
		return ((PoserDataLightGen2 *)pdl)->plane;
		break;
	default:
		assert(0);
	}
	return 0;
}