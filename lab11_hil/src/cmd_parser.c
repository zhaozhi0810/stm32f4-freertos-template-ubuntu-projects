#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>
#include <inttypes.h> // For PRIxx and SCNxx macros

#include "stm32f4xx_hal.h"
#include "cmd_line_buffer.h"
#include "cmd_parser.h"

#include "heartbeat_task.h"
#include "mod_inputs.h"
#include "mod_fault_detection.h"
#include "mod_state_machine.h"
#include "mod_comms.h"
#include "mod_outputs.h"

// Type for each command table entry
typedef struct
{
    void (*func)(int argc, char *argv[]);   // Command function pointer
    const char * cmd;                       // Command name
    const char * args;                      // Command arguments syntax
    const char * help;                      // Command description
} CMD_T;

// Command function declarations
static void _cmd_help(int, char *[]);
static void _cmd_heartbeat(int, char *[]);
static void _cmd_inputs(int, char *[]);
static void _cmd_start(int, char *[]);
static void _cmd_reset(int, char *[]);
static void _cmd_ipt_start(int, char *[]);
static void _cmd_ipt_stop(int, char *[]);
static void _cmd_fd_start(int, char *[]);
static void _cmd_fd_stop(int, char *[]);
static void _cmd_fd_pressure(int, char *[]);
static void _cmd_fd_temperature(int, char *[]);
static void _cmd_fd_estop(int, char *[]);
static void _cmd_fd_error(int, char *[]);
static void _cmd_sm_start(int, char *[]);
static void _cmd_sm_stop(int, char *[]);
static void _cmd_sm_error(int, char *[]);
static void _cmd_opt_start(int, char *[]);
static void _cmd_opt_stop(int, char *[]);
static void _cmd_opt_state(int, char *[]);
static void _cmd_identify(int, char *[]);

// Command table
static CMD_T cmd_table[] =
{
    {_cmd_help,             "help",             "",             "Displays this help message"},
    {_cmd_heartbeat,        "heartbeat",        "[start|stop]", "Get status or start/stop heartbeat task"},
    {_cmd_identify,         "identify",         "",             "Identify student STM. Returns 'student'"},
    {_cmd_start,            "start",            "",             "Start signal to the system by calling module_comms_start_received()."},
    {_cmd_reset,            "reset",            "",             "Restarts the system."},
    {_cmd_ipt_start,        "ipt_start",        "",             "Starts the input module thread by calling module_inputs_start()."},
    {_cmd_ipt_stop,         "ipt_stop",         "",             "Starts the input module thread by calling module_inputs_stop()."},
    {_cmd_inputs,           "ipt_get_values",   "",             "Displays the current input values to the console by calling module_fault_detect_print_inputs()."},
    {_cmd_fd_start,         "fd_start",         "",             "Starts the fault detection module thread by calling module_fault_detect_start()."},
    {_cmd_fd_stop,          "fd_stop",          "",             "Stops the fault detection module thread by calling module_fault_detect_stop()."},
    {_cmd_fd_pressure,      "fd_set_pressure",  "[%f]",         "Set pressure value entering the fault detection module by calling module_fault_detect_manual_pressure()."},
    {_cmd_fd_temperature,   "fd_set_temp",      "[%f]",         "Set temperature value entering the fault detection module by calling module_fault_detect_manual_temperature()"},
    {_cmd_fd_estop,         "fd_set_estop",     "[%i]",         "Set estop state entering the fault detection module by calling module_fault_detect_manual_estop(). 0=HEALTHY, 1=FAULT."},
    {_cmd_fd_error,         "fd_set_error",     "[%i]",         "Set error state entering the fault detection module by calling module_fault_detect_manual_error()."},
    {_cmd_sm_start,         "sm_start",         "",             "Starts the state machine module thread by calling module_state_machine_start()."},
    {_cmd_sm_stop,          "sm_stop",          "",             "Stops the state machine module thread by calling module_state_machine_stop()."},
    {_cmd_sm_error,         "sm_set_error",     "[%i]",         "Set the error state entering the state machine by calling module_state_machine_manual_error()."},
    {_cmd_opt_start,        "opt_start",        "",             "Starts the outputs module thread by calling module_outputs_start()."},
    {_cmd_opt_stop,         "opt_stop",         "",             "Stops the outputs module thread by calling module_outputs_stop()."},
    {_cmd_opt_state,        "opt_set_state",    "[%i]",         "Set the state of the output module by calling module_outputs_manual_state(). 0=OPEN, 1=CLOSED."},
};
enum {CMD_TABLE_SIZE = sizeof(cmd_table)/sizeof(CMD_T)};
enum {CMD_MAX_TOKENS = 5};      // Maximum number of tokens to process (command + arguments)


// Command function definitions
void _cmd_ipt_start(int argc, char *argv[])
{
    UNUSED(argc);
    UNUSED(argv);
    
    module_inputs_start();
}

void _cmd_ipt_stop(int argc, char *argv[])
{
    UNUSED(argc);
    UNUSED(argv);
    
    module_inputs_stop();
}

void _cmd_fd_start(int argc, char *argv[])
{
    UNUSED(argc);
    UNUSED(argv);
    
    module_fault_detect_start();
}

void _cmd_fd_stop(int argc, char *argv[])
{
    UNUSED(argc);
    UNUSED(argv);
    
    module_fault_detect_stop();
}

void _cmd_fd_estop(int argc, char *argv[])
{
    if (argc == 2)
    {
        // Set estop value
        module_fault_detect_manual_estop(atoi(argv[1]));
    }
    else
    {
        printf("Incorrect number of inputs\n");
    }
}

void _cmd_fd_error(int argc, char *argv[])
{
    if (argc == 2)
    {
        // Set error code
        module_fault_detect_manual_error(atoi(argv[1]));
    }
    else
    {
        printf("Incorrect number of inputs\n");
    }
}

void _cmd_fd_pressure(int argc, char *argv[])
{
    if (argc == 2)
    {
        // Set pressure value
        module_fault_detect_manual_pressure(atof(argv[1]));
    }
    else
    {
        printf("Incorrect number of inputs\n");
    }
}

void _cmd_fd_temperature(int argc, char *argv[])
{
    if (argc == 2)
    {
        // Set temperature value
        module_fault_detect_manual_temperature(atof(argv[1]));
    }
    else
    {
        printf("Incorrect number of inputs\n");
    }
}

void _cmd_sm_start(int argc, char *argv[])
{
    UNUSED(argc);
    UNUSED(argv);
    
    module_state_machine_start();
}

void _cmd_sm_stop(int argc, char *argv[])
{
    UNUSED(argc);
    UNUSED(argv);
    
    module_state_machine_stop();
}

void _cmd_sm_error(int argc, char *argv[])
{
    UNUSED(argc);
    
    // Set input fault status
    module_state_machine_manual_error(atoi(argv[1]));
}

void _cmd_opt_start(int argc, char *argv[])
{
    UNUSED(argc);
    UNUSED(argv);
    
    module_outputs_start();
}

void _cmd_opt_stop(int argc, char *argv[])
{
    UNUSED(argc);
    UNUSED(argv);
    
    module_outputs_stop();
}

void _cmd_opt_state(int argc, char *argv[])
{
    UNUSED(argc);
    
    // Set the state
    module_outputs_manual_state(atoi(argv[1]));
}

void _cmd_reset(int argc, char *argv[])
{
    UNUSED(argc);
    UNUSED(argv);
    
    // Reset the system
    HAL_NVIC_SystemReset();
}

void _cmd_start(int argc, char *argv[])
{
    UNUSED(argc);
    UNUSED(argv);
    
    // trigger startReceived function in comms module
    module_comms_start_received();
}

void _cmd_inputs(int argc, char *argv[])
{
    UNUSED(argc);
    UNUSED(argv);
    
    module_fault_detect_print_inputs();
}

void _cmd_identify(int argc, char *argv[])
{
    UNUSED(argc);
    UNUSED(argv);
    
    //To help figure out which com port is our aircon STM or the student's STM without manually unplugging the usb cables
    printf("student\n");
}

void _cmd_heartbeat(int argc, char *argv[])
{
    if (argc <= 1)
    {
        if (heartbeat_task_is_running())
            printf("Heartbeat is currently running\n");
        else
            printf("Heartbeat is not currently running\n");
    }
    else
    {
        if (strcmp(argv[1], "start") == 0)
        {
            heartbeat_task_start();
            printf("Heartbeat has started\n");
        }
        else if (strcmp(argv[1], "stop") == 0)
        {
            heartbeat_task_stop();
            printf("Heartbeat has stopped\n");
        }
        else
        {
            printf("%s: invalid argument \"%s\", syntax is: %s [start|stop]\n", argv[0], argv[1], argv[0]);
        }
    }
}

static void _print_chip_pinout(void);

void _cmd_help(int argc, char *argv[])
{
    UNUSED(argv); // Avoid compiler warning about unused variable
    printf(
        "\n"
        "\n"
    );

    _print_chip_pinout();
    
    printf("\n");

    // Describe argument syntax using POSIX.1-2008 convention
    // see http://pubs.opengroup.org/onlinepubs/9699919799/basedefs/V1_chap12.html
    // printf(
    //     "Available commands:\n"
    //     "    help                       Print this help message\n"
    //     "    pot                        Get potentiometer ADC value\n"
    //     "    enc [reset]                Get or reset encoder count\n"
    //     "    light [<hue> <sat> <val>]  Get or set light HSV values\n"
    //     "    dimmer [start|stop]        Get status or start/stop dimmer task\n"
    //     "    heartbeat [start|stop]     Get status or start/stop heartbeat task\n"
    //     "\n"
    // );
    switch (argc)
    {
    case 1:
        printf(
            "   Command Arguments            Description\n"
            "-------------------------------------------\n"
        );
        for (int i = 0; i < CMD_TABLE_SIZE; i++)
        {
            printf("%10s %-20s %s\n", cmd_table[i].cmd, cmd_table[i].args, cmd_table[i].help);
        }
        // printf("\nFor more information, enter help followed by the command name\n\n");
        break;
    // case 2:
    //     printf("To be implemented...\n\n");
        // TODO: Scan command table, and lookup extended help string.
        break;
    // default:
        // printf("help is expecting zero or one argument.\n\n");
    }
}

void _print_chip_pinout(void)
{
    printf(
        "Pin configuration:\n"
        "\n"
        "       .---------------------------------------.\n"
        " PC10--|  1  2 --PC11              PC9--  1  2 |--PC8\n"
        " PC12--|  3  4 --PD2               PB8--  3  4 |--PC6\n"
        "  VDD--|  5  6 --E5V               PB9--  5  6 |--PC5\n"
        "BOOT0--|  7  8 --GND              AVDD--  7  8 |--U5V\n"
        "   NC--|  9 10 --NC                GND--  9 10 |--NC\n"
        "   NC--| 11 12 --IOREF             PA5-- 11 12 |--PA12\n"
        " PA13--| 13 14 --RESET             PA6-- 13 14 |--PA11\n"
        " PA14--| 15 16 --+3v3              PA7-- 15 16 |--PB12\n"
        " PA15--| 17 18 --+5v               PB6-- 17 18 |--NC\n"
        "  GND--| 19 20 --GND               PC7-- 19 20 |--GND\n"
        "  PB7--| 21 22 --GND               PA9-- 21 22 |--PB2\n"
        " PC13--| 23 24 --VIN               PA8-- 23 24 |--PB1\n"
        " PC14--| 25 26 --NC               PB10-- 25 26 |--PB15\n"
        " PC15--| 27 28 --PA0               PB4-- 27 28 |--PB14\n"
        "  PH0--| 29 30 --PA1               PB5-- 29 30 |--PB13\n"
        "  PH1--| 31 32 --PA4               PB3-- 31 32 |--AGND\n"
        " VBAT--| 33 34 --PB0              PA10-- 33 34 |--PC4\n"
        "  PC2--| 35 36 --PC1               PA2-- 35 36 |--NC\n"
        "  PC3--| 37 38 --PC0               PA3-- 37 38 |--NC\n"
        "       |________                   ____________|\n"
        "                \\_________________/\n"
    );
}

// Command parser

static int _makeargv(char *s, char *argv[], int argvsize);

#ifdef NO_LD_WRAP
void cmd_parse(char *) __asm__("___real_cmd_parse");
#endif

void cmd_parse(char * cmd)
{
    if (cmd == NULL)
    {
        printf("ERROR: Tried to parse NULL command pointer\n");
        return;
    }
    else if (*cmd == '\0') // Empty command string
    {
        return;
    }

    // Tokenise command string
    char *argv[CMD_MAX_TOKENS];
    int argc = _makeargv(cmd, argv, CMD_MAX_TOKENS);

    // Execute corresponding command function
    for (int i = 0; i < CMD_TABLE_SIZE; i++)
    {
        if (strcmp(argv[0], cmd_table[i].cmd) == 0)
        {
            cmd_table[i].func(argc, argv);
            return;
        }
    }

    // Command not found
    printf("Unknown command: \"%s\"\n", argv[0]);
    // if (argc > 1)
    // {
    //     printf("    with arguments:\n");
    //     for (int i = 1; i < argc; i++)
    //     {
    //         printf("        %d: %s\n", i, argv[i]);
    //     }
    // }
    // printf_P(PSTR("\nEnter \"help\" for a list of commands.\n\n"));
}

int _makeargv(char *s, char *argv[], int argvsize)
{
    char *p = s;
    int argc = 0;

    for(int i = 0; i < argvsize; ++i)
    {
        // skip leading whitespace
        while (isspace(*p))
            p++;

        if(*p != '\0')
            argv[argc++] = p;
        else
        {
            argv[argc] = NULL;
            break;
        }

        // scan over arg
        while(*p != '\0' && !isspace(*p))
            p++;

        // terminate arg
        if(*p != '\0' && i < argvsize - 1)
            *p++ = '\0';
    }

    return argc;
}


