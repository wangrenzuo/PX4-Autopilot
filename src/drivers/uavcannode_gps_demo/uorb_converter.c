/*
 * uorb_converter.c
 *
 *  Created on: Apr 10, 2020
 *      Author: hovergames
 */

#include <string.h>

#include "uorb_converter.h"



//GPS message Definitions
#include <legacy/equipment/gnss/Fix2_1_0.h>
#include <legacy/equipment/gnss/Auxiliary_1_0.h>



/****************************************************************************
 * Private Data
 ****************************************************************************/
int uorb_sub_fd;
px4_pollfd_struct_t fds[1];
int error_counter;
CanardInstance *canard_ins;
int fix_message_transfer_id;
int aux_message_transfer_id;
int16_t* gps_fix_port_id;
int16_t* gps_aux_port_id;


void uorbConverterInit(CanardInstance *ins, int16_t *fix_port_id, int16_t *aux_port_id)
{
	canard_ins = ins;
    
    gps_fix_port_id = fix_port_id;
    gps_aux_port_id = aux_port_id;

	/* subscribe to the uORB topic */
	uorb_sub_fd = orb_subscribe(ORB_ID(sensor_gps));

	orb_set_interval(uorb_sub_fd, 200);

	/* one could wait for multiple topics with this technique, just using one here */
	fds[0].fd = uorb_sub_fd;
	fds[0].events = POLLIN;

	error_counter = 0;
	fix_message_transfer_id = 0;
    aux_message_transfer_id = 0;
}

void uorbProcessSub(int timeout_msec)
{
	/* wait for subscriber update of 1 file descriptor for timeout_msec ms */
	int poll_ret = px4_poll(fds, 1, timeout_msec);

	/* handle the poll result */
	if (poll_ret == 0) {
		/* this means none of our providers is giving us data */
	} else if (poll_ret < 0) {
		/* this is seriously bad - should be an emergency */
		if (error_counter < 10 || error_counter % 50 == 0) {
			/* use a counter to prevent flooding (and slowing us down) */
			PX4_ERR("ERROR return value from poll(): %d", poll_ret);
		}

		error_counter++;

	} else {
		if (fds[0].revents & POLLIN) {
			/* obtained data for the first file descriptor */
			struct sensor_gps_s raw;
			/* copy sensors raw data into local buffer */
			orb_copy(ORB_ID(sensor_gps), uorb_sub_fd, &raw);

			CanardMicrosecond transmission_deadline = getMonotonicTimestampUSec() + 1000 * 10;

            //legacy.equipment.gnss.fix2 message
            if(*gps_fix_port_id != -1){
                uint8_t gps_fix_payload_buffer[legacy_equipment_gnss_Fix2_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_];
                
                CanardTransfer transfer = {
                    .timestamp_usec = transmission_deadline,      // Zero if transmission deadline is not limited.
                    .priority       = CanardPriorityNominal,
                    .transfer_kind  = CanardTransferKindMessage,
                    .port_id        = *gps_fix_port_id,                       // This is the subject-ID.
                    .remote_node_id = CANARD_NODE_ID_UNSET,       // Messages cannot be unicast, so use UNSET.
                    .transfer_id    = fix_message_transfer_id,
                    .payload_size   = legacy_equipment_gnss_Fix2_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_,
                    .payload        = &raw,
                };

                legacy_equipment_gnss_Fix2_1_0 gps_fix;
                
                //Conversion magic here
                
                //TODO syncronized timestamp_usec
                
                gps_fix.gnss_timestamp.microsecond = raw.time_utc_usec;
                
                gps_fix.gnss_time_standard = legacy_equipment_gnss_Fix2_1_0_GNSS_TIME_STANDARD_UTC;
                /* gps_fix.num_leap_seconds */
                
                gps_fix.longitude_deg_1e8    = raw.lat; // No conversion need I think TODO check doc
                gps_fix.latitude_deg_1e8     = raw.lon; // No conversion need I think TODO check doc
                gps_fix.height_ellipsoid_mm  = raw.alt_ellipsoid; // No conversion need I think TODO check doc
                gps_fix.height_msl_mm        = raw.alt; // No conversion need I think TODO check doc
                
                gps_fix.ned_velocity[0] = raw.vel_n_m_s; // TODO North?
                gps_fix.ned_velocity[0] = raw.vel_e_m_s; // TODO East ?
                gps_fix.ned_velocity[0] = raw.vel_d_m_s; // TODO Down ?
                
                gps_fix.sats_used = raw.satellites_used;
                
                /** 0-1: no fix, 2: 2D fix, 3: 3D fix, 4: RTCM code differential, 5: Real-Time Kinematic, float, 6: Real-Time Kinematic, fixed, 8: Extrapolated. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.*/
                
                if(raw.fix_type == 2 || raw.fix_type == 3) {
                    gps_fix.status = raw.fix_type;
                } else {
                    gps_fix.status = 0;
                }
                
                /* gps_fix.mode */
                /* gps_fix.sub_mode */
                /* gps_fix.covariance */
                /* gps_fix.pdop */
                
                legacy_equipment_gnss_Fix2_1_0_serialize_(&gps_fix, gps_fix_payload_buffer, &transfer.payload_size);
                
                

                ++fix_message_transfer_id;  // The transfer-ID shall be incremented after every transmission on this subject.
                int32_t result = canardTxPush(canard_ins, &transfer);

                if (result < 0) {
                    // An error has occurred: either an argument is invalid or we've ran out of memory.
                    // It is possible to statically prove that an out-of-memory will never occur for a given application if the
                    // heap is sized correctly; for background, refer to the Robson's Proof and the documentation for O1Heap.
                    PX4_ERR("canardTxPush error %d", result);
                }
            }
			
            //legacy.equipment.gnss.Auxiliary message
            if(*gps_aux_port_id != -1){
                uint8_t gps_aux_payload_buffer[legacy_equipment_gnss_Auxiliary_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_];
                
                CanardTransfer transfer = {
                    .timestamp_usec = transmission_deadline,      // Zero if transmission deadline is not limited.
                    .priority       = CanardPriorityNominal,
                    .transfer_kind  = CanardTransferKindMessage,
                    .port_id        = *gps_aux_port_id,                       // This is the subject-ID.
                    .remote_node_id = CANARD_NODE_ID_UNSET,       // Messages cannot be unicast, so use UNSET.
                    .transfer_id    = aux_message_transfer_id,
                    .payload_size   = legacy_equipment_gnss_Auxiliary_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_,
                    .payload        = &raw,
                };

                legacy_equipment_gnss_Auxiliary_1_0 gps_aux;
                
                //Conversion magic here
                gps_aux.hdop = raw.hdop;
                gps_aux.vdop = raw.vdop;
                gps_aux.sats_used = raw.satellites_used;
                /* gps_aux.sats_visible */
                
                legacy_equipment_gnss_Auxiliary_1_0_serialize_(&gps_aux, gps_aux_payload_buffer, &transfer.payload_size);
                
                

                ++aux_message_transfer_id;  // The transfer-ID shall be incremented after every transmission on this subject.
                int32_t result = canardTxPush(canard_ins, &transfer);

                if (result < 0) {
                    // An error has occurred: either an argument is invalid or we've ran out of memory.
                    // It is possible to statically prove that an out-of-memory will never occur for a given application if the
                    // heap is sized correctly; for background, refer to the Robson's Proof and the documentation for O1Heap.
                    PX4_ERR("canardTxPush error %d", result);
                }
            }

			PX4_INFO("Recevied data from uORB topic");
		}
	}
}

