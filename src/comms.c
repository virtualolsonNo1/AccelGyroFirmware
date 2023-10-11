#include "lsm6dso32.h"
#include "arm_common_tables.h"
#include "arm_math.h"
#include "arm_math_types.h"
#include "dsp/fast_math_functions.h"
#include "dsp/support_functions.h"
#include "dsp/utils.h"
#include "common.pb.h"
#include "pb_encode.h"
#include "pb_decode.h"
#include "viccom.h"
#include "pb.h"
#include "node.pb.h"

const float32_t ACCEL_CONTRIB = 0.01;
const float32_t GYRO_CONTRIB = 1 - ACCEL_CONTRIB;
const float32_t DT = 1/833.;

pb_byte_t dataBuffer[256];

extern viccom_t comms;
extern NodeState state;


void UART0_IRQHandler() {
    VICCOM_uartHandler(&comms);
}

void PDMA_IRQHandler() {
    VICCOM_pdmaHandler(&comms);
}

void EINT1_IRQHandler() {
    //TODO: Will need to discard 1 accel and 3 gyroscope samples right off bat according to the datasheet
    //TODO: make everything reset if interrupt pin never resets to high. Can see examples of what to do with timeout in ethernet-pod

    //buffers for data
    uint8_t lsmDataBuff[12];

    //clear interrupt source flag
    PB->INTSRC |= GPIO_INTSRC_INTSRC4_Msk;

    //read status register to find out what kind of data is available
    uint8_t statusReg = readReg(STATUS_REG);
    volatile bool accelDataAvailable = statusReg & XLDA;
    volatile bool gyroDataAvailable = statusReg & GDA;

    if(accelDataAvailable && gyroDataAvailable) {
        lsm6dso32_getData(lsmDataBuff);
        data_t lsmData;

        //concatenate low and high x, y, and z values to make two's complement
        lsmData.accel_x = lsmDataBuff[X_L_A] | (lsmDataBuff[X_H_A] << HIGH_OFFSET);
        lsmData.accel_y = lsmDataBuff[Y_L_A] | (lsmDataBuff[Y_H_A] << HIGH_OFFSET);
        lsmData.accel_z = lsmDataBuff[Z_L_A] | (lsmDataBuff[Z_H_A] << HIGH_OFFSET);

        lsmData.gyro_x = lsmDataBuff[X_L_G] | (lsmDataBuff[X_H_G] << HIGH_OFFSET);
        lsmData.gyro_y = lsmDataBuff[Y_L_G] | (lsmDataBuff[Y_H_G] << HIGH_OFFSET);
        lsmData.gyro_z = lsmDataBuff[Z_L_G] | (lsmDataBuff[Z_H_G] << HIGH_OFFSET);

        //get accel in gs
        state.imu.accel.x = lsmData.accel_x * ACCEL_RES;
        state.imu.accel.y = lsmData.accel_y * ACCEL_RES;
        state.imu.accel.z = lsmData.accel_z * ACCEL_RES;

        //get gyro in degrees
        state.imu.gyro.x = lsmData.gyro_x * GYRO_RES * PI / 180;
        state.imu.gyro.y = lsmData.gyro_y * GYRO_RES * PI / 180;
        state.imu.gyro.z = lsmData.gyro_z * GYRO_RES * PI / 180;

        //calculate roll and pitch
        {
            float32_t xznorm;
            arm_sqrt_f32(state.imu.gyro.x * state.imu.accel.x + state.imu.accel.z * state.imu.accel.z, &xznorm);
            
            float32_t phi;
            arm_atan2_f32(state.imu.accel.y, xznorm, &phi);
            
            float32_t yznorm;
            arm_sqrt_f32(state.imu.accel.y * state.imu.accel.y + state.imu.accel.z * state.imu.accel.z, &yznorm);

            float32_t theta;
            arm_atan2_f32(-state.imu.accel.x, yznorm, &theta);

            float32_t phi_dot = state.imu.gyro.x + sinf(state.imu.roll) * tanf(state.imu.pitch) * state.imu.gyro.y + cosf(state.imu.roll) * tanf(state.imu.pitch) * state.imu.gyro.z;
            float32_t theta_dot = cosf(state.imu.roll) * state.imu.gyro.y - sinf(state.imu.roll) * state.imu.gyro.z;
            
            state.imu.roll = (state.imu.roll + phi_dot * DT) * GYRO_CONTRIB + phi * ACCEL_CONTRIB;
            state.imu.pitch = (state.imu.pitch + theta_dot * DT) * GYRO_CONTRIB + theta * ACCEL_CONTRIB;
        }


        state.imu.has_accel = 1;
        state.imu.has_gyro = 1;


    //TODO: probably replace these with error or something as need both
    } else if (accelDataAvailable) {
        lsm6dso32_getData(lsmDataBuff);

    } else if (gyroDataAvailable) {
        lsm6dso32_getData(lsmDataBuff);
    }
    return;
}

void COMMS_task(void) {
    if(VICCOM_dataAvailable(&comms)  && !comms.txBusy) {
        // read, decode, and merge in the state
        viccom_rx_t rx = VICCOM_getBuffer(&comms);
        pb_istream_t rx_stream = pb_istream_from_buffer(rx.data, rx.length);
        pb_decode_ex(&rx_stream, NodeState_fields, &state, PB_DECODE_NOINIT);

        // send back our updated state
        pb_ostream_t tx_stream = pb_ostream_from_buffer(dataBuffer, sizeof(dataBuffer));
        pb_encode(&tx_stream, NodeState_fields, &state);
        VICCOM_send(&comms, dataBuffer, tx_stream.bytes_written);

        VICCOM_send(&comms, dataBuffer, tx_stream.bytes_written);
    }

}