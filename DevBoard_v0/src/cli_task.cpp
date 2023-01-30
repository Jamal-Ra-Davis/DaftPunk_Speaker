#include <Arduino.h>
#include "cli_task.h"
#include "global_defines.h"
#include "Logging.h"
#include "I2C_Helper.h"
#include "Volume.h"
#include <SimpleCLI.h>
#include <Adafruit_MAX1704X.h>

#define CLI_TASK_STACK_SIZE 3072

enum CLI_COMMANDS {
    HELP_CMD, 
    PING_CMD, 
    STACK_DISP_CMD, 
    I2C_READ_CMD,
    I2C_WRITE_CMD,
    I2C_BUS_SCAN_CMD,
    MEM_READ_CMD,
    MEM_WRITE_CMD,
    GPIO_READ_CMD,
    GPIO_WRITE_CMD,
    TASK_ENABLE_CMD,
    GET_ENABLED_TASKS_CMD,
    VOL_SET_CMD,
    VOL_GET_CMD,
    BATT_GET_STATUS_CMD,
    BATT_GET_ALERT_CMD,
    BATT_SET_ALERT_CMD,
    RGB_LED_SET_CMD,
    LOG_LEVEL_SET_CMD,
    NUM_CLI_COMMANDS,
};

// File Globals
static SimpleCLI cli;
static TaskHandle_t xcli_task = NULL;
static Command cmdPing;
static Command cmdHelp;
static Command cmdStackDisplay;
extern bool display_stack_wm;
extern Adafruit_MAX17048 maxlipo;
extern int8_t volume_level;
static Command cmd_list[NUM_CLI_COMMANDS];

// Function Prototypes
static void cli_task(void *pvParameters);
static void error_cb(cmd_error* e);

static void help_cb(cmd* c);
static void ping_cb(cmd* c);
static void stack_display_cb(cmd* c);
static void i2c_read_cb(cmd* c);
static void i2c_write_cb(cmd* c);
static void i2c_bus_scan_cb(cmd* c);
static void mem_read_cb(cmd* c);
static void mem_write_cb(cmd* c);
static void gpio_read_cb(cmd* c);
static void gpio_write_cb(cmd* c);
static void task_enable_cb(cmd* c);
static void get_enabled_tasks_cb(cmd* c);
static void vol_set_cb(cmd* c);
static void vol_get_cb(cmd* c);
static void batt_get_status_cb(cmd* c);
static void batt_get_alert_cb(cmd* c);
static void batt_set_alert_cb(cmd* c);
static void rgb_led_set_cb(cmd* c);
static int parse_integer_param(const char *param, int len, int32_t *out);
static int parse_integer_arg(Argument *arg, int32_t *out);
static int buf_to_hex_string(uint8_t *buf, uint8_t buf_len, char *str_buf, uint8_t str_buf_len);

/*
 * TODO: Add in following CLI functions
 *  - I2C read/write
 *  - I2C Bus Scan
 *  - Memory read/write
 *  - GPIO read/write
 *  - Task enable/disable
 *  - Volume control
 *  - Battery status (voltage, current, SOC)
 *  - Battery alerts (set/get)
 *  - RGB LED control (switch between manual control and FW)
 *  - *Print current stack usage    
 *  
**/

// Public Functions
/**
 * @brief Initializes cli task. 
 * @return 0 on success, -1 on failure
 */ 
int init_cli_task()
{
    // Help Command
    cmd_list[HELP_CMD] = cli.addCommand("help", help_cb);
    cmd_list[HELP_CMD].setDescription("\tGet help!");

    // Ping Command
    cmd_list[PING_CMD] = cli.addCmd("ping", ping_cb);
    

    // Stack Display Command
    cmd_list[STACK_DISP_CMD] = cli.addCmd("stack_display", stack_display_cb);
    cmd_list[STACK_DISP_CMD].addPositionalArgument("enable");
    cmd_list[STACK_DISP_CMD].setDescription("\tEnables/Disables stack monitor logs");

    // I2C read
    cmd_list[I2C_READ_CMD] = cli.addCmd("i2c_read", i2c_read_cb);
    cmd_list[I2C_READ_CMD].addPositionalArgument("ADDR");
    cmd_list[I2C_READ_CMD].addPositionalArgument("RDLEN");
    cmd_list[I2C_READ_CMD].addPositionalArgument("REG");
    cmd_list[I2C_READ_CMD].setDescription("\tReads from specified I2C address");

    // I2C write
    cmd_list[I2C_READ_CMD] = cli.addCmd("i2c_write", i2c_write_cb);
    cmd_list[I2C_READ_CMD].addPositionalArgument("ADDR");
    cmd_list[I2C_READ_CMD].addPositionalArgument("REG");
    cmd_list[I2C_READ_CMD].addPositionalArgument("VAL");
    cmd_list[I2C_READ_CMD].setDescription("\tWrite to specified I2C address");

    // I2C Bus Scan
    cmd_list[I2C_BUS_SCAN_CMD] = cli.addCmd("i2c_bus_scan", i2c_bus_scan_cb);
    cmd_list[I2C_BUS_SCAN_CMD].setDescription("\tReturns list of I2C devices on bus");

    // Memory read
    cmd_list[MEM_READ_CMD] = cli.addCmd("mem_read", mem_read_cb);
    cmd_list[MEM_READ_CMD].addPositionalArgument("ADDR");
    cmd_list[MEM_READ_CMD].setDescription("\tReads from specified memory address (must be 4 byte aligned)");

    // Memory write
    cmd_list[MEM_WRITE_CMD] = cli.addCmd("mem_write", mem_write_cb);
    cmd_list[MEM_WRITE_CMD].addPositionalArgument("ADDR");
    cmd_list[MEM_WRITE_CMD].addPositionalArgument("VAL");
    cmd_list[MEM_WRITE_CMD].setDescription("\tWrites value to specified memory address (must be 4 byte aligned)");

    // GPIO read
    cmd_list[GPIO_READ_CMD] = cli.addCmd("gpio_read", gpio_read_cb);
    cmd_list[GPIO_READ_CMD].addPositionalArgument("PIN");
    cmd_list[GPIO_READ_CMD].setDescription("\tReads state of specified gpio pin");

    // GPIO write
    cmd_list[GPIO_WRITE_CMD] = cli.addCmd("gpio_write", gpio_write_cb);
    cmd_list[GPIO_WRITE_CMD].addPositionalArgument("PIN");
    cmd_list[GPIO_WRITE_CMD].addPositionalArgument("VAL");
    cmd_list[GPIO_WRITE_CMD].setDescription("\tSets the state of specified gpio pin");

    // Task enable
    cmd_list[TASK_ENABLE_CMD] = cli.addCmd("task_enable", task_enable_cb);
    cmd_list[TASK_ENABLE_CMD].addPositionalArgument("TASK_ID");
    cmd_list[TASK_ENABLE_CMD].addPositionalArgument("EN");
    cmd_list[TASK_ENABLE_CMD].setDescription("\tEnables/Disabled specified task ID");

    // Get enabled tasks
    cmd_list[GET_ENABLED_TASKS_CMD] = cli.addCmd("get_enabled_tasks", get_enabled_tasks_cb);
    cmd_list[GET_ENABLED_TASKS_CMD].setDescription("\tReturns list of task statuses");

    // Volume set
    cmd_list[VOL_SET_CMD] = cli.addCmd("vol_set", vol_set_cb);
    cmd_list[VOL_SET_CMD].addPositionalArgument("VOL");
    cmd_list[VOL_SET_CMD].setDescription("\tSets volume level");

    // Volume get
    cmd_list[VOL_GET_CMD] = cli.addCmd("vol_get", vol_get_cb);
    cmd_list[VOL_GET_CMD].setDescription("\tReturns current volume level");

    // Battery status (voltage, current, SOC)
    cmd_list[BATT_GET_STATUS_CMD] = cli.addCmd("batt_get_status", batt_get_status_cb);
    cmd_list[BATT_GET_STATUS_CMD].setDescription("\tReturns status of battery (voltage, current, SOC)");

    // Get Battery alerts
    cmd_list[BATT_GET_ALERT_CMD] = cli.addCmd("batt_get_alert", batt_get_alert_cb);
    cmd_list[BATT_GET_ALERT_CMD].setDescription("\tReturns battery alert status");

    // Set Battery alerts
    cmd_list[BATT_SET_ALERT_CMD] = cli.addCmd("batt_set_alert", batt_set_alert_cb);
    cmd_list[BATT_SET_ALERT_CMD].addPositionalArgument("ALRT");
    cmd_list[BATT_SET_ALERT_CMD].setDescription("\tSets battery alerts");

    // RGB LED control (switch between manual control and FW)
    cmd_list[RGB_LED_SET_CMD] = cli.addCmd("rgb_led_set", rgb_led_set_cb);
    cmd_list[RGB_LED_SET_CMD].addPositionalArgument("MODE");
    cmd_list[RGB_LED_SET_CMD].setDescription("\tSets LED mode and control");

    cli.setOnError(error_cb);

    xTaskCreate(
        cli_task,
        "CLI_Task",
        CLI_TASK_STACK_SIZE,
        NULL,
        CLI_TASK_PRIORITY,
        &xcli_task);
    return 0;
}
TaskHandle_t cli_task_handle()
{
    return xcli_task;
}

// Private Functions
static void cli_task(void *pvParameters)
{
    while (1)
    {
        if (Serial.available()) {
            String input = Serial.readStringUntil('\n');

            if (input.length() > 0) {
                Serial.print("# ");
                Serial.println(input);

                cli.parse(input);
            }
        }
        else {
            delay(100);
        }
    }
}
static void error_cb(cmd_error* e) {
    CommandError cmdError(e); // Create wrapper object

    Serial.print("ERROR: ");
    Serial.println(cmdError.toString());

    if (cmdError.hasCommand()) {
        Serial.print("Did you mean \"");
        Serial.print(cmdError.getCommand().toString());
        Serial.println("\"?");
    }
}
static void help_cb(cmd* c) {
    Command cmd(c);

    Serial.println("Help:");
    Serial.println(cli.toString());
}
static void ping_cb(cmd* c) {
    Command cmd(c);

    Serial.println("Pong!");
}
static void stack_display_cb(cmd* c) {
    Command cmd(c);

    Argument enable_arg = cmd.getArgument("enable");
    bool enable = (bool)enable_arg.getValue().toInt();
    display_stack_wm = enable;
}
static void i2c_read_cb(cmd* c)
{
    log_inf("i2c_read");
    Command cmd(c);
    bool valid_inputs = true;
    uint8_t rbuf[16];
    
    //address
    Argument addr_arg = cmd.getArgument("ADDR");
    int32_t addr_val;

    //num bytes
    Argument rdlen_arg = cmd.getArgument("RDLEN");
    int32_t rdlen_val;

    //wbuf (reg addr)
    Argument reg_arg = cmd.getArgument("REG");
    int32_t reg_val;

    
    if (parse_integer_arg(&addr_arg, &addr_val) < 0) {
        log_err("Failed to parse i2c address");
        valid_inputs = false;
    }
    if (parse_integer_arg(&rdlen_arg, &rdlen_val) < 0) {
        log_err("Failed to parse i2c read length");
        valid_inputs = false;
    }
    if (parse_integer_arg(&reg_arg, &reg_val) < 0) {
        log_err("Failed to parse i2c reg");
        valid_inputs = false;
    }
    if (rdlen_val > sizeof(rbuf)) {
        log_err("Bytes requested (%d) exceeds max (%d)", rdlen_val, sizeof(rbuf));
        valid_inputs = false;
    } 
    if (!valid_inputs) {
        return;
    }
    uint8_t _reg = (uint8_t)reg_val;
    if (i2c_write_read((uint8_t)addr_val, &_reg, sizeof(uint8_t), rbuf, rdlen_val) < 0) {
        log_err("Failed to read from device 0x%02X at register 0x%02X", (uint8_t)addr_val, _reg);
        return;
    }
    log_inf("addr_val = 0x%02X (%d)", addr_val, addr_val);

    char result_str[64];
    if (buf_to_hex_string(rbuf, rdlen_val, result_str, sizeof(result_str)) < 0) {
        log_err("Failed to build output string");
        return;
    }
    log_inf("I2C[0x%02X][0x%02X]: %s", (uint8_t)addr_val, _reg, result_str);
}
static void i2c_write_cb(cmd* c)
{
    log_inf("i2c_write");
    Command cmd(c);
    bool valid_inputs = true;

    //address
    Argument addr_arg = cmd.getArgument("ADDR");
    int32_t addr_val;

    //wbuf (reg addr)
    Argument reg_arg = cmd.getArgument("REG");
    int32_t reg_val;

    //Write value
    Argument val_arg = cmd.getArgument("VAL");
    int32_t val_val;
    int byte_cnt = 0;

    if (parse_integer_arg(&addr_arg, &addr_val) < 0) {
        log_err("Failed to parse i2c address");
        valid_inputs = false;
    }
    if (parse_integer_arg(&reg_arg, &reg_val) < 0) {
        log_err("Failed to parse i2c reg");
        valid_inputs = false;
    }
    if (parse_integer_arg(&val_arg, &val_val) < 0) {
        log_err("Failed to parse i2c read length");
        valid_inputs = false;
    }
    if (val_val < 0) {
        valid_inputs = false;
    }

    if (!valid_inputs) {
        return;
    }

    if (val_val > 0) {
        int32_t _val = val_val;
        while (_val > 0) {
            _val >>= 8;
            byte_cnt++;
        }
    }
    else {
        byte_cnt = 1;
    }
    
    uint8_t _reg = (uint8_t)reg_val;
    if (i2c_write((uint8_t)addr_val, (uint8_t*)&val_val, byte_cnt) < 0) {
        log_err("Failed to perform i2c write to device 0x%02X at register 0x%02X", (uint8_t)addr_val, _reg);
        return;
    }
}
static void i2c_bus_scan_cb(cmd* c)
{
    log_inf("i2c_bus_scan");
    uint8_t addr_list[8];
    uint8_t addr_cnt;

    char addr_str[64] = {'\0'};
    if (i2c_bus_scan(addr_list, &addr_cnt, sizeof(addr_list)) < 0) {
        log_err("I2C bus scan failed");
        return;
    }

    if (buf_to_hex_string(addr_list, addr_cnt, addr_str, sizeof(addr_str)) < 0) {
        log_err("Failed to build output string");
        return;
    }
    log_inf("I2C Addresses: %s", addr_str);
}
static void mem_read_cb(cmd* c)
{
    log_inf("mem_read");
    Command cmd(c);
    bool valid_inputs = true;

    //address
    Argument addr_arg = cmd.getArgument("ADDR");
    int32_t addr_val;

    if (parse_integer_arg(&addr_arg, &addr_val) < 0) {
        log_err("Failed to parse memory address");
        valid_inputs = false;
    }
    if (addr_val < 0) {
        valid_inputs = false;
    }

    if (!valid_inputs) {
        return;
    }

    int32_t mem_val = *((int32_t*)addr_val);
    log_inf("MEM[0x%08X] = 0x%08X (%d)", mem_val, mem_val);
}
static void mem_write_cb(cmd* c)
{
    log_inf("mem_write");
    Command cmd(c);
    bool valid_inputs = true;

    //address
    Argument addr_arg = cmd.getArgument("ADDR");
    int32_t addr_val;

    //Write value
    Argument val_arg = cmd.getArgument("VAL");
    int32_t val_val;

    if (parse_integer_arg(&addr_arg, &addr_val) < 0) {
        log_err("Failed to parse memory address");
        valid_inputs = false;
    }
    if (parse_integer_arg(&val_arg, &val_val) < 0) {
        log_err("Failed to parse memory value");
        valid_inputs = false;
    }
    if (addr_val < 0) {
        valid_inputs = false;
    }

    if (!valid_inputs) {
        return;
    }

    *((int32_t*)addr_val) = val_val;
}
static void gpio_read_cb(cmd* c)
{
    log_inf("gpio_read");
    Command cmd(c);
    bool valid_inputs = true;

    //address
    Argument pin_arg = cmd.getArgument("PIN");
    int32_t pin_val;

    if (parse_integer_arg(&pin_arg, &pin_val) < 0) {
        log_err("Failed to parse gpio pin");
        valid_inputs = false;
    }
    if (pin_val < 0) {
        valid_inputs = false;
    }

    if (!valid_inputs) {
        return;
    }

    log_inf("GPIO[%d] =%d", pin_val, digitalRead(pin_val));
}
static void gpio_write_cb(cmd* c)
{
    log_inf("gpio_write");
    Command cmd(c);
    bool valid_inputs = true;

    //address
    Argument pin_arg = cmd.getArgument("PIN");
    int32_t pin_val;

    //Write value
    Argument val_arg = cmd.getArgument("VAL");
    int32_t val_val;

    if (parse_integer_arg(&pin_arg, &pin_val) < 0) {
        log_err("Failed to parse gpio pin");
        valid_inputs = false;
    }
    if (parse_integer_arg(&val_arg, &val_val) < 0) {
        log_err("Failed to parse pin setting value");
        valid_inputs = false;
    }
    if (pin_val < 0 || val_val < 0) {
        valid_inputs = false;
    }

    if (!valid_inputs) {
        return;
    }

    digitalWrite(pin_val, (bool)val_val);
}
static void task_enable_cb(cmd* c)
{
    log_inf("task_enable");
}
static void get_enabled_tasks_cb(cmd* c)
{
    log_inf("get_enabled_tasks");
}
static void vol_set_cb(cmd* c)
{
    log_inf("vol_set");
    Command cmd(c);
    bool valid_inputs = true;

    //address
    Argument vol_arg = cmd.getArgument("VOL");
    int32_t vol_val;

    if (parse_integer_arg(&vol_arg, &vol_val) < 0) {
        log_err("Failed to parse volume value");
        return;
    }
    
    log_dbg("Volume Val = %d", vol_val);
    if (volume_set((int8_t)vol_val) < 0) {
        log_err("Failed to set volume");
        return;
    }
}
static void vol_get_cb(cmd* c)
{
    log_inf("vol_get");
    log_inf("Volume Level: %d", volume_get());
}
static void batt_get_status_cb(cmd* c)
{
    log_inf("batt_get_status");
    float voltage = maxlipo.cellVoltage();
    float soc = maxlipo.cellPercent();
    log_inf("Battery Voltage: %0.2fV, Battery SOC: %0.2f %%", voltage, soc);
}
static void batt_get_alert_cb(cmd* c)
{
    log_inf("batt_get_alert");
}
static void batt_set_alert_cb(cmd* c)
{
    log_inf("batt_set_alert");
}
static void rgb_led_set_cb(cmd* c)
{
    log_inf("rgb_led_set");
}


static int parse_integer_param(const char *param, int len, int32_t *out)
{
    char data_buf[16] = {'\0'};
    uint32_t value;
    if (param == NULL) {
        return -1;
    }
    if (len > 15) {
        return -1;
    }

    memcpy(data_buf, param, len);

    if (data_buf[1] == 'x')
    {
        value = (uint32_t)strtol(data_buf, NULL, 0);
    }
    else
    {
        value = atoi(data_buf);
    }
    *out = value;
    return 0;
}
static int parse_integer_arg(Argument *arg, int32_t *out)
{
    char buf[16] = {'\0'};
    String arg_str = arg->getValue();
    int arg_len = arg_str.length();

    if (arg_len > 15) {
        return -1;
    }

    arg_str.toCharArray(buf, arg_len+1);
    return parse_integer_param(buf, arg_len, out);
}
static int buf_to_hex_string(uint8_t *buf, uint8_t buf_len, char *str_buf, uint8_t str_buf_len)
{
    if (buf == NULL || str_buf == NULL) {
        return -1;
    }
    if (buf_len == 0 || str_buf_len == 0) {
        return -1;
    }
    static const uint8_t HEX_AND_COMMA_LEN = 6;
    static const uint8_t LAST_VAL_AND_BRACKETS = 7;
    int bytes_needed = LAST_VAL_AND_BRACKETS + HEX_AND_COMMA_LEN*(buf_len-1);
    if (str_buf_len < bytes_needed) {
        return -1;
    }

    uint8_t str_idx = 0;
    str_buf[str_idx++] = '[';
    for (int i=0; i<buf_len; i++) {
        int offset = snprintf(&str_buf[str_idx], str_buf_len - str_idx, "0x%02X", buf[i]);
        str_idx += offset;

        if (i < buf_len - 1) {
            str_buf[str_idx++] = ',';
            str_buf[str_idx++] = ' ';
        }
    }
    str_buf[str_idx++] = ']';
    str_buf[str_idx++] = '\0';
    return 0;
}