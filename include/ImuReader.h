// Parker Stebbins, March 2013

#ifndef _ImuReader_h_
	#define _ImuReader_h_

	#include "Arduino.h"
	#include "Wire.h"
	#include "math3D.h"
	#include "I2Cdev.h"
	#include "MPU6050_9Axis_MotionControlApps41.h"

	struct Frame {
		float ypr[3] = {0.0, 0.0, 0.0};
		float ypr_velocity[3] = {0.0, 0.0, 0.0};
		float ypr_acceleration[3] = {0.0, 0.0, 0.0};

		Vector3 <float> position(0.0, 0.0, 0.0);
		Vector3 <float> velocity(0.0, 0.0, 0.0);
		Vector3 <float> acceleration(0.0, 0.0, 0.0);
	};

	class StateReader {
		public:
			StateReader() {
				Wire.begin();
				mpu.initialize();
				mpu.testConnection();
				devStatus = mpu.dmpInitialize();
				
				if (devStatus == 0) {
					mpu.setDMPEnabled(true);
					attachInterrupt(0, dmpDataReady, RISING);
					mpuIntStatus = mpu.getIntStatus();
					dmpReady = true;
					packetSize = mpu.dmpGetFIFOPacketSize();
				}
			}

			bool read(uint elapsed) {
				if (!dmpReady) return false;
				while (!mpuInterrupt && fifoCount < packetSize);

				mpuInterrupt = false;
				mpuIntStatus = mpu.getIntStatus();
				fifoCount = mpu.getFIFOCount();

				if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
					mpu.resetFIFO();
					return false;
				} else if (mpuIntStatus & 0x02) {
					while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

					mpu.getFIFOBytes(fifoBuffer, packetSize);
					fifoCount -= packetSize;

					mpu.dmpGetQuaternion(&q, fifoBuffer);
					mpu.dmpGetGravity(&gravity, &q);
				}

				float _elapsed = elapsed / 1000;
				prev_state = state;

				mpu.dmpGetYawPitchRoll(state.ypr, &q, &gravity);
				for (uint8_t i = 0; i <= 2; i++) {
					state.ypr_velocity[i] = (state.ypr[i] - prev_state.ypr[i]) / _elapsed;
					state.ypr_acceleration[i] = (state.ypr_velocity[i] - prev_state.ypr_velocity[i]) / _elapsed;
				}

				lVector3 <int16_t> _acc;
				mpu.dmpGetAccel(&_acc, fifoBuffer);
				state.acceleration.x = (highByte(_acc.x) << 8) | lowByte(_acc.x);
				state.acceleration.y = (highByte(_acc.y) << 8) | lowByte(_acc.y);
				state.acceleration.z = (highByte(_acc.z) << 8) | lowByte(_acc.z);		
				DELETE _acc;

				state.velocity += state.acceleration;
				state.position += state.velocity;
				return true;
			}

			Frame state;
			

		private:
			Frame prev_state;

			bool dmpReady = false; // set true if DMP init was successful
			uint8_t mpuIntStatus; // holds actual interrupt status byte from MPU
			uint8_t devStatus; // return status after each device operation (0 = success, !0 = error)
			uint16_t packetSize; // expected DMP packet size (default is 42 bytes)
			uint16_t fifoCount; // count of all bytes currently in FIFO
			uint8_t fifoBuffer[64]; // FIFO storage buffer
			volatile bool mpuInterrupt = false;

			void dmpDataReady() {
				mpuInterrupt = true;
			}

			Quaternion q(1.0, 0.0, 0.0, 0.0);
			Vector3 <float> gravity(0.0, 0.0, 0.0);
	};
#endif