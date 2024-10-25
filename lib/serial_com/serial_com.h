#ifndef SERIAL_COM_H
#define SERIAL_COM_H

#include <Arduino.h>

#define MAX_COMMAND_SIZE 24
#define MAX_FLAG_SIZE 24
#define MAX_FLAGS 5

class flag{
    String flag_name[MAX_FLAG_SIZE];
    float flag_value;
};

class serial_command {
private:
    String main_command[MAX_COMMAND_SIZE];
    flag flags[MAX_FLAGS];
public:
    void parse_command(String command);
    void parse_flags();

    String get_main_command_name();
    flag get_flag_value(String flag_name);
    
};

#endif  // SERIAL_COM_H