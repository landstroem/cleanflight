/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <stdbool.h>

#include <limits.h>

extern "C" {
    #include "debug.h"

    #include <platform.h>
    #include "build_config.h"

    #include "common/axis.h"
    #include "common/maths.h"
    #include "common/filter.h"

    #include "config/parameter_group.h"
    #include "config/parameter_group_ids.h"

    #include "drivers/sensor.h"
    #include "drivers/accgyro.h"
    #include "drivers/pwm_mapping.h"
    #include "drivers/gyro_sync.h"

    #include "sensors/sensors.h"
    #include "sensors/acceleration.h"

    #include "rx/rx.h"
    #include "flight/pid.h"
    #include "flight/imu.h"
    #include "flight/mixer.h"
    #include "flight/servos.h"

    #include "io/motor_and_servo.h"
    #include "io/gimbal.h"
    #include "io/rc_controls.h"

    #include "config/config.h"

    extern uint8_t servoCount;
    void forwardAuxChannelsToServos(uint8_t firstServoIndex);

    void mixerInit(motorMixer_t *initialCustomMixers);
    void mixerInitServos(servoMixer_t *initialCustomServoMixers);
    void mixerUsePWMIOConfiguration(pwmIOConfiguration_t *pwmIOConfiguration);

    PG_REGISTER_PROFILE(gimbalConfig_t, gimbalConfig, PG_GIMBAL_CONFIG, 0);
    PG_REGISTER(motorAndServoConfig_t, motorAndServoConfig, PG_MOTOR_AND_SERVO_CONFIG, 0);
    PG_REGISTER(rxConfig_t, rxConfig, PG_RX_CONFIG, 0);

    PG_REGISTER_PROFILE(rcControlsConfig_t, rcControlsConfig, PG_RC_CONTROLS_CONFIG, 0);
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

// input
#define TEST_RC_MID 1500

// output
#define TEST_MIN_COMMAND 1000
#define TEST_SERVO_MID 1500

typedef struct motor_s {
    uint16_t value;
} motor_t;

typedef struct servo_s {
    uint16_t value;
} servo_t;

motor_t motors[MAX_SUPPORTED_MOTORS];
servo_t servos[MAX_SUPPORTED_SERVOS];

uint8_t lastOneShotUpdateMotorCount;

uint32_t testFeatureMask = 0;

int updatedServoCount;
int updatedMotorCount;

TEST(FlightAxisUnittest, TestAxisIndices)
{
    // In various places Cleanflight assumes equality between the flight dynamics indices,
    // and angle indices, the RC controls indices, and the PID indices.
    // This test asserts those equalities.

    // check the FD indices have the correct absolute values
    EXPECT_EQ(0, FD_ROLL);
    EXPECT_EQ(1, FD_PITCH);
    EXPECT_EQ(2, FD_YAW);
    EXPECT_EQ(3, FD_INDEX_COUNT);

    // check the AI indices match the FD indices
    EXPECT_EQ(FD_ROLL, AI_ROLL);
    EXPECT_EQ(FD_PITCH, AI_PITCH);
    EXPECT_EQ(2, ANGLE_INDEX_COUNT);

    // check the RC indices match the FD indices
    EXPECT_EQ(FD_ROLL, ROLL);
    EXPECT_EQ(FD_PITCH, PITCH);
    EXPECT_EQ(FD_YAW, YAW);
    EXPECT_EQ(3, THROTTLE); // throttle is sometimes used the fourth item in a zero based array

    // check the PID indices match the FD indices
    EXPECT_EQ(FD_ROLL, PIDROLL);
    EXPECT_EQ(FD_PITCH, PIDPITCH);
    EXPECT_EQ(FD_YAW, PIDYAW);
}

class ChannelForwardingTest : public ::testing::Test {
protected:
    virtual void SetUp() {
        memset(&servos, 0, sizeof(servos));
    }
};


TEST_F(ChannelForwardingTest, TestForwardAuxChannelsToServosWithNoServos)
{
    // given
    servoCount = 0;

    rcData[AUX1] = TEST_RC_MID;
    rcData[AUX2] = TEST_RC_MID;
    rcData[AUX3] = TEST_RC_MID;
    rcData[AUX4] = TEST_RC_MID;

    // when
    forwardAuxChannelsToServos(MAX_SUPPORTED_SERVOS);

    // then
    for (uint8_t i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        EXPECT_EQ(0, servos[i].value);
    }
}

TEST_F(ChannelForwardingTest, TestForwardAuxChannelsToServosWithMaxServos)
{
    // given
    servoCount = MAX_SUPPORTED_SERVOS;

    rcData[AUX1] = 1000;
    rcData[AUX2] = 1250;
    rcData[AUX3] = 1750;
    rcData[AUX4] = 2000;

    // when
    forwardAuxChannelsToServos(MAX_SUPPORTED_SERVOS);

    // then
    uint8_t i;
    for (i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        EXPECT_EQ(0, servos[i].value);
    }
}

TEST_F(ChannelForwardingTest, TestForwardAuxChannelsToServosWithLessRemainingServosThanAuxChannelsToForward)
{
    // given
    servoCount = MAX_SUPPORTED_SERVOS - 2;

    rcData[AUX1] = 1000;
    rcData[AUX2] = 1250;
    rcData[AUX3] = 1750;
    rcData[AUX4] = 2000;

    // when
    forwardAuxChannelsToServos(MAX_SUPPORTED_SERVOS - 2);

    // then
    uint8_t i;
    for (i = 0; i < MAX_SUPPORTED_SERVOS - 2; i++) {
        EXPECT_EQ(0, servos[i].value);
    }

    // -1 for zero based offset
    EXPECT_EQ(1000, servos[MAX_SUPPORTED_SERVOS - 1 - 1].value);
    EXPECT_EQ(1250, servos[MAX_SUPPORTED_SERVOS - 0 - 1].value);
}

class BasicMixerIntegrationTest : public ::testing::Test {
protected:
    servoParam_t servoConf[MAX_SUPPORTED_SERVOS];
    gimbalConfig_t gimbalConfig = {
        .mode = GIMBAL_MODE_NORMAL
    };

    virtual void SetUp() {
        updatedServoCount = 0;
        updatedMotorCount = 0;

        memset(mixerConfig(), 0, sizeof(*mixerConfig()));
        memset(rxConfig(), 0, sizeof(*rxConfig()));
        memset(motorAndServoConfig(), 0, sizeof(*motorAndServoConfig()));
        memset(servoProfile(), 0, sizeof(*servoProfile()));

        memset(rcData, 0, sizeof(rcData));
        memset(rcCommand, 0, sizeof(rcCommand));
        memset(axisPID, 0, sizeof(axisPID));
        memset(customMotorMixer_arr(), 0, sizeof(*customMotorMixer_arr()));
    }

    virtual void withDefaultmotorAndServoConfiguration(void) {
        motorAndServoConfig()->mincommand = TEST_MIN_COMMAND;
    }

    virtual void withDefaultRxConfig(void) {
        rxConfig()->midrc = 1500;
    }
    
    virtual void withDefaultExistingServoConfig(servoParam_t *servoParam) {
        servoParam->min = DEFAULT_SERVO_MIN;
        servoParam->max = DEFAULT_SERVO_MAX;
        servoParam->middle = DEFAULT_SERVO_MIDDLE;
        servoParam->rate = 100;
        servoParam->forwardFromChannel = CHANNEL_FORWARDING_DISABLED;
    }

    virtual void configureMixer(uint8_t mixerMode) {
        mixerConfig()->mixerMode = mixerMode;
        mixerUseConfigs(
            servoConf
        );
    }
};

TEST_F(BasicMixerIntegrationTest, TestTricopterServo)
{
    // given
    rxConfig()->midrc = 1500;

    mixerConfig()->tri_unarmed_servo = 1;

    withDefaultmotorAndServoConfiguration();
    withDefaultRxConfig();

    withDefaultExistingServoConfig(&servoConf[SERVO_RUDDER]);

    configureMixer(MIXER_TRI);

    mixerInit(customMotorMixer(0));
    mixerInitServos(customServoMixer(0));

    // and
    pwmIOConfiguration_t pwmIOConfiguration = {
            .servoCount = 1,
            .motorCount = 3,
            .ioCount = 4,
            .pwmInputCount = 0,
            .ppmInputCount = 0,
            .ioConfigurations = {}
    };

    mixerUsePWMIOConfiguration(&pwmIOConfiguration);

    // and
    axisPID[FD_YAW] = 0;

    // when
    mixTable();
    writeServos();

    // then
    EXPECT_EQ(1, updatedServoCount);
    EXPECT_EQ(TEST_SERVO_MID, servos[0].value);
}

TEST_F(BasicMixerIntegrationTest, TestQuadMotors)
{
    // given
    withDefaultmotorAndServoConfiguration();

    configureMixer(MIXER_QUADX);

    mixerInit(customMotorMixer(0));
    mixerInitServos(customServoMixer(0));

    // and
    pwmIOConfiguration_t pwmIOConfiguration = {
            .servoCount = 0,
            .motorCount = 4,
            .ioCount = 4,
            .pwmInputCount = 0,
            .ppmInputCount = 0,
            .ioConfigurations = {}
    };

    mixerUsePWMIOConfiguration(&pwmIOConfiguration);

    // and
    memset(rcCommand, 0, sizeof(rcCommand));

    // and
    memset(axisPID, 0, sizeof(axisPID));
    axisPID[FD_YAW] = 0;


    // when
    mixTable();
    writeMotors();

    // then
    EXPECT_EQ(4, updatedMotorCount);

    EXPECT_EQ(TEST_MIN_COMMAND, motors[0].value);
    EXPECT_EQ(TEST_MIN_COMMAND, motors[1].value);
    EXPECT_EQ(TEST_MIN_COMMAND, motors[2].value);
    EXPECT_EQ(TEST_MIN_COMMAND, motors[3].value);
}

class FlyingWingServoMixerIntegrationTest : public BasicMixerIntegrationTest {
protected:
    //  Input range [-500:+500]
    const int16_t input = 100; 

    // Test passes for current implementation,
    // FIXME but should we not expect (input / 2) for each mix?
    const int16_t expectedSinglePositiveMixedInputs = DEFAULT_SERVO_MIDDLE + input;
    const int16_t expectedSingleNegativeMixedInputs = DEFAULT_SERVO_MIDDLE - input;
    
    const int16_t expectedTwoPositiveMixedInputs = DEFAULT_SERVO_MIDDLE + 2 * input;
    const int16_t expectedTwoNegativeMixedInputs = DEFAULT_SERVO_MIDDLE - 2 * input;
    
    virtual void SetUp() {
        BasicMixerIntegrationTest::SetUp();

        rxConfig()->midrc = TEST_RC_MID;

        withDefaultmotorAndServoConfiguration();
        withDefaultRxConfig();

        // Left elevon
        withDefaultExistingServoConfig(&servoConf[SERVO_FLAPPERON_1]);

        // Right elevon
        withDefaultExistingServoConfig(&servoConf[SERVO_FLAPPERON_2]);
        
        // Adjust the default assumptions for the servo physical installation so
        // that a positive input gets translated into a increased servo ouutput value
        servoConf[SERVO_FLAPPERON_1].rate = -100;
        servoConf[SERVO_FLAPPERON_2].rate = 100;
        
        configureMixer(MIXER_FLYING_WING);

        mixerInit(customMotorMixer(0));
        mixerInitServos(customServoMixer(0));

        pwmIOConfiguration_t pwmIOConfiguration = {
                .servoCount = 2,
                .motorCount = 2,
                .ioCount = 4,
                .pwmInputCount = 0,
                .ppmInputCount = 0,
                .ioConfigurations = {}
        };

        mixerUsePWMIOConfiguration(&pwmIOConfiguration);
    }
};

TEST_F(FlyingWingServoMixerIntegrationTest, TestPositivePitchInput)
{
    // given
    axisPID[PITCH] = input;
    
    // when
    mixTable();
    writeServos();

    // then
    EXPECT_EQ(2, updatedServoCount);
    EXPECT_EQ(expectedSinglePositiveMixedInputs, servos[0].value) << "for SERVO_FLAPPERON_1";
    EXPECT_EQ(expectedSinglePositiveMixedInputs, servos[1].value) << "for SERVO_FLAPPERON_2";
}

TEST_F(FlyingWingServoMixerIntegrationTest, TestNegativePitchInput)
{
    // given
    axisPID[PITCH] = -input;
    
    // when
    mixTable();
    writeServos();

    // then
    EXPECT_EQ(2, updatedServoCount);
    EXPECT_EQ(expectedSingleNegativeMixedInputs, servos[0].value) << "for SERVO_FLAPPERON_1";
    EXPECT_EQ(expectedSingleNegativeMixedInputs, servos[1].value) << "for SERVO_FLAPPERON_2";
}

TEST_F(FlyingWingServoMixerIntegrationTest, TestPositiveRollInput)
{
    // given
    axisPID[ROLL] = input;
    
    // when
    mixTable();
    writeServos();

    // then
    EXPECT_EQ(2, updatedServoCount);
    EXPECT_EQ(expectedSingleNegativeMixedInputs, servos[0].value) << "for SERVO_FLAPPERON_1";
    EXPECT_EQ(expectedSinglePositiveMixedInputs, servos[1].value) << "for SERVO_FLAPPERON_2";
}

TEST_F(FlyingWingServoMixerIntegrationTest, TestNegativeRollInput)
{
    // given
    axisPID[ROLL] = -input;
    
    // when
    mixTable();
    writeServos();

    // then
    EXPECT_EQ(2, updatedServoCount);
    EXPECT_EQ(expectedSinglePositiveMixedInputs, servos[0].value) << "for SERVO_FLAPPERON_1";
    EXPECT_EQ(expectedSingleNegativeMixedInputs, servos[1].value) << "for SERVO_FLAPPERON_2";
}

TEST_F(FlyingWingServoMixerIntegrationTest, TestPositivePitchAndRollInput)
{
    // given
    axisPID[PITCH] = input;
    axisPID[ROLL] = input;
    
    // when
    mixTable();
    writeServos();

    // then
    EXPECT_EQ(2, updatedServoCount);
    EXPECT_EQ(DEFAULT_SERVO_MIDDLE, servos[0].value) << "for SERVO_FLAPPERON_1";
    EXPECT_EQ(expectedTwoPositiveMixedInputs, servos[1].value) << "for SERVO_FLAPPERON_2";

    // FIXME ...but this will only roll right, not pitch up?
}

TEST_F(FlyingWingServoMixerIntegrationTest, TestNegativePitchInputAndPositiveRollInput)
{
    // given
    axisPID[PITCH] = -input;
    axisPID[ROLL] = input;
    
    // when
    mixTable();
    writeServos();

    // then
    EXPECT_EQ(2, updatedServoCount);
    EXPECT_EQ(expectedTwoNegativeMixedInputs, servos[0].value) << "for SERVO_FLAPPERON_1";
    EXPECT_EQ(DEFAULT_SERVO_MIDDLE, servos[1].value) << "for SERVO_FLAPPERON_2";

    // FIXME ...but this will only roll left, not pitch down?
}

class CustomMixerIntegrationTest : public BasicMixerIntegrationTest {
protected:

    virtual void SetUp() {

        BasicMixerIntegrationTest::SetUp();

        memset(&servoConf, 0, sizeof(servoConf));
        for (int i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
            withDefaultExistingServoConfig(&servoConf[i]);
        }

        withDefaultmotorAndServoConfiguration();
        withDefaultRxConfig();

        configureMixer(MIXER_QUADX);

        memset(*customMotorMixer_arr(), 0, sizeof(*customMotorMixer_arr()));
        memset(*customServoMixer_arr(), 0, sizeof(*customServoMixer_arr()));
    }
};


TEST_F(CustomMixerIntegrationTest, TestCustomMixer)
{
    // given
    enum {
        EXPECTED_SERVOS_TO_MIX_COUNT = 6,
        EXPECTED_MOTORS_TO_MIX_COUNT = 2
    };

    servoMixer_t testServoMixer[EXPECTED_SERVOS_TO_MIX_COUNT] = {
        { SERVO_ELEVATOR, INPUT_STABILIZED_PITCH, 100, 0, 0, 100, 0 },
        { SERVO_FLAPPERON_1, INPUT_STABILIZED_ROLL,  100, 0, 0, 100, 0 },
        { SERVO_FLAPPERON_2, INPUT_STABILIZED_ROLL,  100, 0, 0, 100, 0 },
        { SERVO_RUDDER, INPUT_STABILIZED_YAW,   100, 0, 0, 100, 0 },
        { SERVO_THROTTLE, INPUT_STABILIZED_THROTTLE, 100, 0, 0, 100, 0 },
        { SERVO_FLAPS, INPUT_RC_AUX1,  100, 0, 0, 100, 0 },
    };
    memcpy(customServoMixer_arr(), testServoMixer, sizeof(testServoMixer));

    static const motorMixer_t testMotorMixer[EXPECTED_MOTORS_TO_MIX_COUNT] = {
        { 1.0f,  0.0f,  0.0f, -1.0f },          // LEFT
        { 1.0f,  0.0f,  0.0f,  1.0f },          // RIGHT
    };
    memcpy(customMotorMixer_arr(), testMotorMixer, sizeof(testMotorMixer));

    configureMixer(MIXER_CUSTOM_AIRPLANE);

    mixerInit(customMotorMixer(0));
    mixerInitServos(customServoMixer(0));

    pwmIOConfiguration_t pwmIOConfiguration = {
            .servoCount = 6,
            .motorCount = 2,
            .ioCount = 8,
            .pwmInputCount = 0,
            .ppmInputCount = 0,
            .ioConfigurations = {}
    };

    mixerUsePWMIOConfiguration(&pwmIOConfiguration);

    // and
    rcCommand[THROTTLE] = 1000;

    // and
    rcData[AUX1] = 2000;

    // and
    memset(axisPID, 0, sizeof(axisPID));
    axisPID[FD_YAW] = 0;


    // when
    mixTable();
    writeMotors();
    writeServos();

    // then
    EXPECT_EQ(EXPECTED_MOTORS_TO_MIX_COUNT, updatedMotorCount);

    EXPECT_EQ(TEST_MIN_COMMAND, motors[0].value);
    EXPECT_EQ(TEST_MIN_COMMAND, motors[1].value);

    EXPECT_EQ(EXPECTED_SERVOS_TO_MIX_COUNT, updatedServoCount);

    EXPECT_EQ(TEST_SERVO_MID, servos[0].value);
    EXPECT_EQ(TEST_SERVO_MID, servos[1].value);
    EXPECT_EQ(TEST_SERVO_MID, servos[2].value);
    EXPECT_EQ(TEST_SERVO_MID, servos[3].value);
    EXPECT_EQ(1000, servos[4].value); // Throttle
    EXPECT_EQ(2000, servos[5].value); // Flaps

}

// STUBS

extern "C" {
attitudeEulerAngles_t attitude;
rxRuntimeConfig_t rxRuntimeConfig;

int16_t axisPID[XYZ_AXIS_COUNT];
int16_t rcCommand[4];
int16_t rcData[MAX_SUPPORTED_RC_CHANNEL_COUNT];

uint32_t rcModeActivationMask;
int16_t debug[DEBUG16_VALUE_COUNT];

uint8_t stateFlags;
uint16_t flightModeFlags;
uint8_t armingFlags;

uint32_t targetLooptime;

void delay(uint32_t) {}

float applyBiQuadFilter(float sample, biquad_t *state) {UNUSED(state);return sample;}
void BiQuadNewLpf(float filterCutFreq, biquad_t *newState, uint32_t refreshRate) {UNUSED(filterCutFreq);UNUSED(newState);UNUSED(refreshRate);}


bool feature(uint32_t mask) {
    return (mask & testFeatureMask);
}

void pwmWriteMotor(uint8_t index, uint16_t value) {
    motors[index].value = value;
    updatedMotorCount++;
}

void pwmShutdownPulsesForAllMotors(uint8_t motorCount)
{
    uint8_t index;

    for(index = 0; index < motorCount; index++){
        motors[index].value = 0;
    }
}

void pwmCompleteOneshotMotorUpdate(uint8_t motorCount) {
    lastOneShotUpdateMotorCount = motorCount;
}

void pwmWriteServo(uint8_t index, uint16_t value) {
    // FIXME logic in test, mimic's production code.
    // Perhaps the solution is to remove the logic from the production code version and assume that
    // anything calling calling pwmWriteServo always uses a valid index?
    // See MAX_SERVOS in pwm_output (driver) and MAX_SUPPORTED_SERVOS (flight)
    if (index < MAX_SERVOS) {
        servos[index].value = value;
    }
    updatedServoCount++;
}

bool rcModeIsActive(boxId_e modeId) { return rcModeActivationMask & (1 << modeId); }

bool failsafeIsActive(void) {
    return false;
}

}
