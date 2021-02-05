#include "MyxaTest.h"

#define ARRAY_SIZE(array)( sizeof( array )/sizeof( array[0]))

#define MYXA_MSG_DELIMITER 0x8E
#define MYXA_MSG_ESCAPE_BYTE 0x9E

//message types
#define MYXA_GENERAL_STATUS_MESSAGE_TYPE 0x00
#define MYXA_DEVICE_CHARACTERISTICS_MESSAGE_TYPE 0x01
#define MYXA_COMMAND_MESSAGE_TYPE 0x02
#define MYXA_TASK_STATISTICS_MESSAGE_TYPE 0x03
#define MYXA_INFO_MESSAGE_TYPE 0xFF


#define MYXA_IDLE_TASK 0x00
#define MYXA_FAULT_TASK 0x01
#define MYXA_BEEP_TASK 0x02
#define MYXA_RUN_TASK 0x03
#define MYXA_HARDWARE_TEST_TASK 0x04
#define MYXA_MOTOR_IDENTIFICATION_TASK 0x05
#define MYXA_LOW_LEVEL_MANIPULATION_TASK 0x06


#define MYXA_CONTROL_MODE_RATIOMETRIC_CURRENT 0x00
#define MYXA_CONTROL_MODE_RATIOMETRIC_RPM 0x01
#define MYXA_CONTROL_MODE_RATIOMETRIC_VOLTAGE 0x02
#define MYXA_CONTROL_MODE_CURRENT 0x03
#define MYXA_CONTROL_MODE_MECHANICAL_RPM 0x04
#define MYXA_CONTROL_MODE_VOLTAGE 0x05


#define MYXA_RUN_TASK_HOLDUP_MS 100
#define MYXA_STATUS_CMD_HOLDUP_MS 500
#define MYXA_INIT_DELAY_MS 3000

#define RX_BUFFER_LENGTH 650

class MyxaESC {
public:
    MyxaESC(HardwareSerial* _comSerialPort, const uint32_t _comSerialBaudRate,
        HardwareSerial* _loggingSerialPort, const uint32_t _loggingSerialBaudRate,
        boolean _loggingEnabled,
        int _statusMsgInterval, int _runMsgInterval) {
        comSerialPort = _comSerialPort;
        loggingSerialPort = _loggingSerialPort;
        loggingEnabled = _loggingEnabled;
        statusMsgIntervalMs = _statusMsgInterval;
        runMsgIntervalMs = _runMsgInterval;
        comSerialPort->begin(_comSerialBaudRate);
        loggingSerialPort->begin(_loggingSerialBaudRate);
    }


    void sendMyxaRunTask(byte motorControlMode, float controlValue) {
        if (millis() > lastRunMsgTimestamp + runMsgIntervalMs) {

            uint8_t payloadAndFrameType[13] = { 0 };
            ((RUN_TASK_PACKET*)payloadAndFrameType)->taskId = MYXA_RUN_TASK;
            ((RUN_TASK_PACKET*)payloadAndFrameType)->mode = motorControlMode;
            ((RUN_TASK_PACKET*)payloadAndFrameType)->value = controlValue;
            ((RUN_TASK_PACKET*)payloadAndFrameType)->frametype = MYXA_COMMAND_MESSAGE_TYPE;

            //crc32c
            byte crc32cBytes[4];
            crc32cBytesLittleEndian(payloadAndFrameType, ARRAY_SIZE(payloadAndFrameType), crc32cBytes);

            sendToMyxa(MYXA_MSG_DELIMITER, false);
            sendToMyxa(payloadAndFrameType, ARRAY_SIZE(payloadAndFrameType), true);
            sendToMyxa(crc32cBytes, ARRAY_SIZE(crc32cBytes), true);
            sendToMyxa(MYXA_MSG_DELIMITER, false);

            lastRunMsgTimestamp = millis();
        }
    }

    void sendMyxaBeepTask(float frequency, float duration) {
        if (millis() > lastRunMsgTimestamp + runMsgIntervalMs) {

            uint8_t payloadAndFrameType[13] = { 0 };
            ((BEEP_TASK_PACKET*)payloadAndFrameType)->taskId = MYXA_BEEP_TASK;
            ((BEEP_TASK_PACKET*)payloadAndFrameType)->freq = frequency;
            ((BEEP_TASK_PACKET*)payloadAndFrameType)->duration = duration;
            ((BEEP_TASK_PACKET*)payloadAndFrameType)->frameType = MYXA_COMMAND_MESSAGE_TYPE;

            byte crc32cBytes[4];
            crc32cBytesLittleEndian(payloadAndFrameType, ARRAY_SIZE(payloadAndFrameType), crc32cBytes);

            sendToMyxa(MYXA_MSG_DELIMITER, false);
            sendToMyxa(payloadAndFrameType, ARRAY_SIZE(payloadAndFrameType), true);
            sendToMyxa(crc32cBytes, ARRAY_SIZE(crc32cBytes), true);
            sendToMyxa(MYXA_MSG_DELIMITER, false);

            lastRunMsgTimestamp = millis();
        }
    }

    void sendMyxaHardwareTestTask(void) {
        uint8_t payloadAndFrameType[5] = { 0 };
        ((TEST_TASK_PACKET*)payloadAndFrameType)->taskId = MYXA_HARDWARE_TEST_TASK;
        ((TEST_TASK_PACKET*)payloadAndFrameType)->frametype = MYXA_COMMAND_MESSAGE_TYPE;

        //crc32c
        byte crc32cBytes[4];
        crc32cBytesLittleEndian(payloadAndFrameType, ARRAY_SIZE(payloadAndFrameType), crc32cBytes);
        sendToMyxa(MYXA_MSG_DELIMITER, false);
        sendToMyxa(payloadAndFrameType, ARRAY_SIZE(payloadAndFrameType), true);
        sendToMyxa(crc32cBytes, ARRAY_SIZE(crc32cBytes), true);
        sendToMyxa(MYXA_MSG_DELIMITER, false);
    }


    void requestMyxaMessage(uint8_t request_type) {
        if (millis() > lastStatusMsgTimestamp + statusMsgIntervalMs) {
            uint8_t frameType = request_type;
            uint8_t payloadAndFrameType[1] = { frameType };

            //crc32c
            byte crc32cBytes[4];
            crc32cBytesLittleEndian(payloadAndFrameType, ARRAY_SIZE(payloadAndFrameType), crc32cBytes);

            sendToMyxa(MYXA_MSG_DELIMITER, false);
            sendToMyxa(payloadAndFrameType, ARRAY_SIZE(payloadAndFrameType), true);
            sendToMyxa( crc32cBytes, ARRAY_SIZE(crc32cBytes), true);
            sendToMyxa( MYXA_MSG_DELIMITER, false);

            lastStatusMsgTimestamp = millis();
            lastRequestMessageType = request_type;
        }
    }
    
        //this is a bit different, uses the Popcop protocol
        void requestMyxaInfo(void) {
        if (millis() > lastStatusMsgTimestamp + statusMsgIntervalMs) {
            uint8_t payloadAndFrameType[3] = { 0x00,0x00,0xFF };

            //crc32c
            byte crc32cBytes[4];
            crc32cBytesLittleEndian(payloadAndFrameType, ARRAY_SIZE(payloadAndFrameType), crc32cBytes);

            sendToMyxa(MYXA_MSG_DELIMITER, false);
            sendToMyxa(payloadAndFrameType, ARRAY_SIZE(payloadAndFrameType), true);
            sendToMyxa( crc32cBytes, ARRAY_SIZE(crc32cBytes), true);
            sendToMyxa( MYXA_MSG_DELIMITER, false);

            lastStatusMsgTimestamp = millis();
            lastRequestMessageType = MYXA_INFO_MESSAGE_TYPE;
        }
    }


    void readMyxaSerial(void) {
        if (comSerialPort->available()) {
            byte msgByte = comSerialPort->read();
            //loggingSerialPort->write(msgByte);
            if (myxaRawMsgLength == 0 && msgByte != MYXA_MSG_DELIMITER) {
                return;
            }

            myxaRawMsg[myxaRawMsgLength++] = msgByte;

            if (msgByte == MYXA_MSG_DELIMITER && myxaRawMsgLength > 1) {
                parseMyxaMessage();
                myxaRawMsgLength = 0;
            }
        }
    }

    //NB! Implementation limited to run task use case
    void parseMyxaMessage(void) {
        byte __attribute__((__may_alias__)) myxaUnescapedMsg[RX_BUFFER_LENGTH];
        boolean unescapeNow = false;
        int unescapedMsgIndex = 0;

        for (int i = 0; i < myxaRawMsgLength; i++) {
            if (myxaRawMsg[i] == MYXA_MSG_ESCAPE_BYTE) {
                unescapeNow = true;
                continue;
            }

            byte msgByte = myxaRawMsg[i];
            if (unescapeNow == true) {
                msgByte = ~msgByte;
                unescapeNow = false;
            }
            myxaUnescapedMsg[unescapedMsgIndex++] = msgByte;
        }
        
        if (loggingEnabled == true) {
            //loggingSerialPort->println("RAW:");
            //printMyxaMessageByteSequence( myxaRawMsg, myxaRawMsgLength );
            //loggingSerialPort->println("UNESCAPED:");
            printMyxaMessageByteSequence( myxaUnescapedMsg, unescapedMsgIndex);
           
            if (lastRequestMessageType == MYXA_GENERAL_STATUS_MESSAGE_TYPE) {
                loggingSerialPort->println("");
                loggingSerialPort->print(", V:");
                loggingSerialPort->print(((GENERAL_STATUS_PACKET*)myxaUnescapedMsg)->dc_voltage);
                loggingSerialPort->print(", I:");
                loggingSerialPort->print(((GENERAL_STATUS_PACKET*)myxaUnescapedMsg)->dc_current);
                loggingSerialPort->print(", LowVolt:");
                loggingSerialPort->print(((GENERAL_STATUS_PACKET*)myxaUnescapedMsg)->lvps_malfunction);
                loggingSerialPort->print(", Overload:");
                loggingSerialPort->print(((GENERAL_STATUS_PACKET*)myxaUnescapedMsg)->overload);
                loggingSerialPort->print(", Fault:");
                loggingSerialPort->print(((GENERAL_STATUS_PACKET*)myxaUnescapedMsg)->fault);

                loggingSerialPort->print(", Task:");
                loggingSerialPort->print(((GENERAL_STATUS_PACKET*)myxaUnescapedMsg)->current_task_id);

                if (((GENERAL_STATUS_PACKET*)myxaUnescapedMsg)->current_task_id == MYXA_RUN_TASK) {
                    loggingSerialPort->print(", CtrlMode:");
                    loggingSerialPort->print(((GENERAL_STATUS_PACKET*)myxaUnescapedMsg)->task_specific_report.run.mode);
                    loggingSerialPort->print(", RPM:");
                    loggingSerialPort->print(((GENERAL_STATUS_PACKET*)myxaUnescapedMsg)->\
                        task_specific_report.run.mechanical_angular_velocity * (60 / (2 * 3.14159))); //convert rad/seconds to RPM
                    loggingSerialPort->print(", torque:");
                    loggingSerialPort->print(((GENERAL_STATUS_PACKET*)myxaUnescapedMsg)->task_specific_report.run.torque);
                    loggingSerialPort->print(", spinup:");
                    loggingSerialPort->print(((GENERAL_STATUS_PACKET*)myxaUnescapedMsg)->task_specific_report.run.spinup_in_progress);
                    loggingSerialPort->print(", reverse:");
                    loggingSerialPort->print(((GENERAL_STATUS_PACKET*)myxaUnescapedMsg)->task_specific_report.run.rotation_reversed);
                    loggingSerialPort->print(", saturated:");
                    loggingSerialPort->print(((GENERAL_STATUS_PACKET*)myxaUnescapedMsg)->task_specific_report.run.controller_saturated);
                }
                loggingSerialPort->println("\n");
            }
            else if (lastRequestMessageType == MYXA_TASK_STATISTICS_MESSAGE_TYPE) {
                for (int i = 0; i < 7; i++) {
                    loggingSerialPort->println("");
                    loggingSerialPort->print("task_id:");
                    loggingSerialPort->print(((TASK_STATS_PACKET*)myxaUnescapedMsg)->task[i].task_id);
                    loggingSerialPort->print(", #started:");
                    loggingSerialPort->print((unsigned long)((TASK_STATS_PACKET*)myxaUnescapedMsg)->task[i].number_of_times_started);
                }
                
            }
            else if (lastRequestMessageType == MYXA_INFO_MESSAGE_TYPE) {
                loggingSerialPort->println("");
                loggingSerialPort->print("device:");
                loggingSerialPort->print(((ENDPOINT_INFO_PACKET*)myxaUnescapedMsg)->endpoint_description);
                loggingSerialPort->print(", software ver:");
                loggingSerialPort->print(((ENDPOINT_INFO_PACKET*)myxaUnescapedMsg)->software_version_major);
                loggingSerialPort->print('.');
                loggingSerialPort->print(((ENDPOINT_INFO_PACKET*)myxaUnescapedMsg)->software_version_minor);
                loggingSerialPort->print(", hardware ver:");
                loggingSerialPort->print(((ENDPOINT_INFO_PACKET*)myxaUnescapedMsg)->hardware_version_major);
                loggingSerialPort->print('.');
                loggingSerialPort->print(((ENDPOINT_INFO_PACKET*)myxaUnescapedMsg)->hardware_version_minor);

            }
        }
    }

    void printMyxaMessageByteSequence( byte myxaUnescapedMsg[], int msgLength) {
        if (loggingEnabled == true) {
            for (int i = 0; i < msgLength; i++) {
                loggingSerialPort->write(myxaUnescapedMsg[i]);
            }
        }
    }

    void sendToMyxa(uint8_t byteVal[], int sizeOfArray, boolean escape) {
        for (int i = 0; i < sizeOfArray; i++) {
            sendToMyxa(byteVal[i], escape);
        }
    }

    void sendToMyxa(uint8_t byteVal, boolean escape) {
        byte byteToSend = byteVal;
        if (escape == true && byteVal == MYXA_MSG_DELIMITER) {
            sendToMyxa( 0x9E, false);
            byteToSend = ~byteVal;
        }
        comSerialPort->write(byteToSend);
    }

    uint32_t crc32c(uint8_t* buf, int len)
    {
        uint32_t crc = 0xffffffff;
        while (len-- > 0) {
            crc = (crc >> 8) ^ crctable[(crc ^ (*buf++)) & 0xFF];
        }
        return crc ^ 0xffffffff;
    };

    void crc32cBytesLittleEndian(uint8_t* buf, int len, byte toByteArray[])
    {
        union {
            uint32_t uint32_crc;
            uint8_t crc_bytes[4];
        } u;
        u.uint32_crc = crc32c(buf, len);
        toByteArray[3] = u.crc_bytes[3];
        toByteArray[2] = u.crc_bytes[2];
        toByteArray[1] = u.crc_bytes[1];
        toByteArray[0] = u.crc_bytes[0];
    };

    void float2BytesLittleEndian(float val, byte* bytes_array) {
        union {
            float float_variable;
            byte temp_array[4];
        } u;
        u.float_variable = val;
        memcpy(bytes_array, u.temp_array, 4);
    }

    float bytes2FloatLittleEndian(byte fourBytes[]) {
        return *((float*)(fourBytes));
    }


private:
    uint32_t crctable[256] = {
      0x00000000L,0xF26B8303L,0xE13B70F7L,0x1350F3F4L,
      0xC79A971FL,0x35F1141CL,0x26A1E7E8L,0xD4CA64EBL,
      0x8AD958CFL,0x78B2DBCCL,0x6BE22838L,0x9989AB3BL,
      0x4D43CFD0L,0xBF284CD3L,0xAC78BF27L,0x5E133C24L,
      0x105EC76FL,0xE235446CL,0xF165B798L,0x030E349BL,
      0xD7C45070L,0x25AFD373L,0x36FF2087L,0xC494A384L,
      0x9A879FA0L,0x68EC1CA3L,0x7BBCEF57L,0x89D76C54L,
      0x5D1D08BFL,0xAF768BBCL,0xBC267848L,0x4E4DFB4BL,
      0x20BD8EDEL,0xD2D60DDDL,0xC186FE29L,0x33ED7D2AL,
      0xE72719C1L,0x154C9AC2L,0x061C6936L,0xF477EA35L,
      0xAA64D611L,0x580F5512L,0x4B5FA6E6L,0xB93425E5L,
      0x6DFE410EL,0x9F95C20DL,0x8CC531F9L,0x7EAEB2FAL,
      0x30E349B1L,0xC288CAB2L,0xD1D83946L,0x23B3BA45L,
      0xF779DEAEL,0x05125DADL,0x1642AE59L,0xE4292D5AL,
      0xBA3A117EL,0x4851927DL,0x5B016189L,0xA96AE28AL,
      0x7DA08661L,0x8FCB0562L,0x9C9BF696L,0x6EF07595L,
      0x417B1DBCL,0xB3109EBFL,0xA0406D4BL,0x522BEE48L,
      0x86E18AA3L,0x748A09A0L,0x67DAFA54L,0x95B17957L,
      0xCBA24573L,0x39C9C670L,0x2A993584L,0xD8F2B687L,
      0x0C38D26CL,0xFE53516FL,0xED03A29BL,0x1F682198L,
      0x5125DAD3L,0xA34E59D0L,0xB01EAA24L,0x42752927L,
      0x96BF4DCCL,0x64D4CECFL,0x77843D3BL,0x85EFBE38L,
      0xDBFC821CL,0x2997011FL,0x3AC7F2EBL,0xC8AC71E8L,
      0x1C661503L,0xEE0D9600L,0xFD5D65F4L,0x0F36E6F7L,
      0x61C69362L,0x93AD1061L,0x80FDE395L,0x72966096L,
      0xA65C047DL,0x5437877EL,0x4767748AL,0xB50CF789L,
      0xEB1FCBADL,0x197448AEL,0x0A24BB5AL,0xF84F3859L,
      0x2C855CB2L,0xDEEEDFB1L,0xCDBE2C45L,0x3FD5AF46L,
      0x7198540DL,0x83F3D70EL,0x90A324FAL,0x62C8A7F9L,
      0xB602C312L,0x44694011L,0x5739B3E5L,0xA55230E6L,
      0xFB410CC2L,0x092A8FC1L,0x1A7A7C35L,0xE811FF36L,
      0x3CDB9BDDL,0xCEB018DEL,0xDDE0EB2AL,0x2F8B6829L,
      0x82F63B78L,0x709DB87BL,0x63CD4B8FL,0x91A6C88CL,
      0x456CAC67L,0xB7072F64L,0xA457DC90L,0x563C5F93L,
      0x082F63B7L,0xFA44E0B4L,0xE9141340L,0x1B7F9043L,
      0xCFB5F4A8L,0x3DDE77ABL,0x2E8E845FL,0xDCE5075CL,
      0x92A8FC17L,0x60C37F14L,0x73938CE0L,0x81F80FE3L,
      0x55326B08L,0xA759E80BL,0xB4091BFFL,0x466298FCL,
      0x1871A4D8L,0xEA1A27DBL,0xF94AD42FL,0x0B21572CL,
      0xDFEB33C7L,0x2D80B0C4L,0x3ED04330L,0xCCBBC033L,
      0xA24BB5A6L,0x502036A5L,0x4370C551L,0xB11B4652L,
      0x65D122B9L,0x97BAA1BAL,0x84EA524EL,0x7681D14DL,
      0x2892ED69L,0xDAF96E6AL,0xC9A99D9EL,0x3BC21E9DL,
      0xEF087A76L,0x1D63F975L,0x0E330A81L,0xFC588982L,
      0xB21572C9L,0x407EF1CAL,0x532E023EL,0xA145813DL,
      0x758FE5D6L,0x87E466D5L,0x94B49521L,0x66DF1622L,
      0x38CC2A06L,0xCAA7A905L,0xD9F75AF1L,0x2B9CD9F2L,
      0xFF56BD19L,0x0D3D3E1AL,0x1E6DCDEEL,0xEC064EEDL,
      0xC38D26C4L,0x31E6A5C7L,0x22B65633L,0xD0DDD530L,
      0x0417B1DBL,0xF67C32D8L,0xE52CC12CL,0x1747422FL,
      0x49547E0BL,0xBB3FFD08L,0xA86F0EFCL,0x5A048DFFL,
      0x8ECEE914L,0x7CA56A17L,0x6FF599E3L,0x9D9E1AE0L,
      0xD3D3E1ABL,0x21B862A8L,0x32E8915CL,0xC083125FL,
      0x144976B4L,0xE622F5B7L,0xF5720643L,0x07198540L,
      0x590AB964L,0xAB613A67L,0xB831C993L,0x4A5A4A90L,
      0x9E902E7BL,0x6CFBAD78L,0x7FAB5E8CL,0x8DC0DD8FL,
      0xE330A81AL,0x115B2B19L,0x020BD8EDL,0xF0605BEEL,
      0x24AA3F05L,0xD6C1BC06L,0xC5914FF2L,0x37FACCF1L,
      0x69E9F0D5L,0x9B8273D6L,0x88D28022L,0x7AB90321L,
      0xAE7367CAL,0x5C18E4C9L,0x4F48173DL,0xBD23943EL,
      0xF36E6F75L,0x0105EC76L,0x12551F82L,0xE03E9C81L,
      0x34F4F86AL,0xC69F7B69L,0xD5CF889DL,0x27A40B9EL,
      0x79B737BAL,0x8BDCB4B9L,0x988C474DL,0x6AE7C44EL,
      0xBE2DA0A5L,0x4C4623A6L,0x5F16D052L,0xAD7D5351L
    };

    HardwareSerial* comSerialPort;
    HardwareSerial* loggingSerialPort;
    boolean loggingEnabled;

    uint16_t statusMsgIntervalMs;
    uint16_t runMsgIntervalMs;
    unsigned long lastStatusMsgTimestamp = millis();
    unsigned long lastRunMsgTimestamp = millis();

    byte myxaRawMsg[RX_BUFFER_LENGTH];
    uint16_t myxaRawMsgLength = 0;
    uint8_t lastRequestMessageType;

};


MyxaESC myxa(&Serial3, 115200, &Serial5, 115200, true, MYXA_STATUS_CMD_HOLDUP_MS, MYXA_RUN_TASK_HOLDUP_MS);
void setup() {
    myxa.sendMyxaHardwareTestTask();
    

}


void loop() {
    //myxa.sendMyxaBeepTask(3000, 1);
    //myxa.sendMyxaRunTask(MYXA_CONTROL_MODE_MECHANICAL_RPM, 500);

    //only send one type of request at a time or parser won't know what to parse.
    //myxa.requestMyxaMessage(MYXA_GENERAL_STATUS_MESSAGE_TYPE);
    //myxa.requestMyxaMessage(MYXA_TASK_STATISTICS_MESSAGE_TYPE);
    myxa.requestMyxaInfo();

    myxa.readMyxaSerial();
}