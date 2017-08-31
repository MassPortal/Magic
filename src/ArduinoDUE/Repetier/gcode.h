/*
    This file is part of Repetier-Firmware.

    Repetier-Firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Repetier-Firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Repetier-Firmware.  If not, see <http://www.gnu.org/licenses/>.

*/
#ifndef _GCODE_H
#define _GCODE_H

#define MAX_CMD_SIZE 96
#define ARRAY_SIZE(_x)	(sizeof(_x)/sizeof(_x[0]))
class GCode   // 52 uint8_ts per command needed
{
    uint16_t params;
    uint16_t params2;
public:
    uint16_t N; // Line number
    uint16_t M;
    uint16_t G;
    float X;
    float Y;
    float Z;
    float E;
    float F;
    int32_t S;
    int32_t P;
    float I;
    float J;
    float R;
    float D;
    float C;
    float H;
    float A;
    float B;
    float K;
    float L;
    float O;

    char *text; //text[17];
    //moved the byte to the end and aligned ints on short boundary
    // Old habit from PC, which require alignments for data types such as int and long to be on 2 or 4 byte boundary
    // Otherwise, the compiler adds padding, wasted space.
    uint8_t T; // This may not matter on any of these controllers, but it can't hurt
    // True if origin did not come from serial console. That way we can send status messages to
    // a host only if he would normally not know about the mode switch.
    bool internalCommand;
    inline bool hasN()
    {
        return (params & (1<<0));
    }
    inline bool hasM()
    {
        return (params & (1<<1));
    }
    inline bool hasG()
    {
        return (params & (1<<2));
    }
    inline bool hasX()
    {
        return (params & (1<<3));
    }
    inline bool hasY()
    {
        return (params & (1<<4));
    }
    inline bool hasZ()
    {
        return (params & (1<<5));
    }
    inline bool hasNoXYZ()
    {
        return !hasX() && !hasY() && !hasZ();
    }
    inline bool hasE()
    {
        return (params & (1<<6));
    }
    inline bool hasF()
    {
        return (params & (1<<8));
    }
    inline bool hasT()
    {
        return (params & (1<<9));
    }
    inline bool hasS()
    {
        return (params & (1<<10));
    }
    inline bool hasP()
    {
        return (params & (1<<11));
    }
    inline bool isV2()
    {
        return (params & (1<<12));
    }
    inline bool hasString()
    {
        return (params & (1<<15));
    }
    inline bool hasI()
    {
        return (params2 & (1<<0));
    }
    inline bool hasJ()
    {
        return (params2 & (1<<1));
    }
    inline bool hasR()
    {
        return (params2 & (1<<2));
    }
    inline bool hasD()
    {
        return (params2 & (1<<3));
    }
    inline bool hasC()
    {
        return (params2 & (1<<4));
    }
    inline bool hasH()
    {
        return (params2 & (1<<5));
    }
    inline bool hasA()
    {
        return (params2 & (1<<6));
    }
    inline bool hasB()
    {
        return (params2 & (1<<7));
    }
    inline bool hasK()
    {
        return (params2 & (1<<8));
    }
    inline bool hasL()
    {
        return (params2 & (1<<9));
    }
    inline bool hasO()
    {
        return (params2 & (1<<10));
    }
    inline long getS(long def)
    {
        return (hasS() ? S : def);
    }
    inline long getP(long def)
    {
        return (hasP() ? P : def);
    }
    inline void setFormatError() {
        params2 |= (1<<15);
    }
    inline bool hasFormatError() {
        return (params2 & (1<<15));
    }
    void printCommand();
    bool parseBinary(uint8_t *buffer,bool fromSerial);
    bool parseAscii(char *line,bool fromSerial);
    void popCurrentCommand();
    void echoCommand();
    /** Get next command in command buffer. After the command is processed, call gcode_command_finished() */
    static GCode *peekCurrentCommand();
    /** Frees the cache used by the last command fetched. */
    static void readFromSerial();
    static void pushCommand();
    static uint8_t computeBinarySize(char *ptr);
private:
    void debugCommandBuffer();
    void checkAndPushCommand();
    static void requestResend();
    inline float parseFloatValue(char *s)
    {
        char *endPtr;
        while(*s == 32) s++; // skip spaces
        float f = (strtod(s, &endPtr));
        if(s == endPtr) f=0.0; // treat empty string "x " as "x0"
        return f;
    }
    inline long parseLongValue(char *s)
    {
        char *endPtr;
        while(*s == 32) s++; // skip spaces
        long l = (strtol(s, &endPtr, 10));
        if(s == endPtr) l=0; // treat empty string argument "p " as "p0"
        return l;
    }
    static GCode commandReceived; ///< Buffer for received commands.
    static bool commandAvailable;
    static uint8_t commandReceiving[MAX_CMD_SIZE]; ///< Current received command.
    static uint8_t commandsReceivingWritePosition; ///< Writing position in gcode_transbuffer.
    static uint8_t sendAsBinary; ///< Flags the command as binary input.
    static uint8_t wasLastCommandReceivedAsBinary; ///< Was the last successful command in binary mode?
    static uint8_t commentDetected; ///< Flags true if we are reading the comment part of a command.
    static uint8_t binaryCommandSize; ///< Expected size of the incoming binary command.
    static uint32_t lastLineNumber; ///< Last line number received.
    static uint32_t actLineNumber; ///< Line number of current command.
    static int8_t waitingForResend; ///< Waiting for line to be resend. -1 = no wait.
    static millis_t timeOfLastDataPacket; ///< Time, when we got the last data packet. Used to detect missing uint8_ts.
    static uint8_t formatErrors; ///< Number of sequential format errors
};

#endif /* _GCODE_H */
