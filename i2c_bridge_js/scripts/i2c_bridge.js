#!/usr/bin/env node
process.env.NODE_PATH = '/home/user/ros2_ws/src/i2c_bridge_js/node_modules';
require('module').Module._initPaths();
const rclnodejs = require('rclnodejs');
const i2c = require('i2c-bus');

const STM32 = 0x3F;
const DAC1  = 0x60;
const DAC2  = 0x61;
const I2C_BUS = 7;

async function main() {
    await rclnodejs.init();
    const node = rclnodejs.createNode('i2c_bridge_js');
    
    let bus;
    try {
        bus = i2c.openSync(I2C_BUS);
        node.getLogger().info(`I2C Bus ${I2C_BUS} opened successfully.`);
    } catch (err) {
        node.getLogger().error(`Failed to open I2C Bus ${I2C_BUS}: ${err.message}`);
    }

    function dacSet(addr, value) {
        if (!bus) return;
        const high = (value >> 8) & 0x0F;
        const low  = value & 0xFF;
        bus.writeI2cBlockSync(addr, high, 1, Buffer.from([low]));
        node.getLogger().info(`DAC [0x${addr.toString(16).toUpperCase()}] set to ${value}`);
    }

    function motor(id, direction, intensity = 255) {
        if (!bus) return;
        const buf = Buffer.from([id.charCodeAt(0), direction, intensity]);
        bus.writeI2cBlockSync(STM32, buf[0], 2, buf.slice(1));
        node.getLogger().info(`Motor ${id} -> direction ${direction}, intensity ${intensity}`);
    }

    // 1. String command (/i2c_cmd) - Legacy string parsing
    node.createSubscription('std_msgs/msg/String', '/i2c_cmd', (msg) => {
        const cmdText = msg.data.trim();
        try {
            const args = cmdText.split(/\s+/);
            const cmd  = args[0].toLowerCase();
            if (cmd === 'dac1' || cmd === 'dac2') {
                const addr = cmd === 'dac1' ? DAC1 : DAC2;
                let val;
                if (args[1] === 'on') val = 4095;
                else if (args[1] === 'off') val = 0;
                else val = parseInt(args[1]);
                if (!isNaN(val)) dacSet(addr, val);
            } else if (cmd === 'motor') {
                const id = args[1].toUpperCase();
                const dir = parseInt(args[2]);
                const int = args[3] ? parseInt(args[3]) : 255;
                if (!isNaN(dir)) motor(id, dir, int);
            } else if (cmd === 'raw') {
                const addr = parseInt(args[1]);
                const reg = parseInt(args[2]);
                const data = args.slice(3).map(x => parseInt(x));
                if (!isNaN(addr) && bus) {
                    bus.writeI2cBlockSync(addr, reg, data.length, Buffer.from(data));
                    node.getLogger().info(`Raw write to 0x${addr.toString(16)}: [${data}]`);
                }
            }
        } catch (e) {
            node.getLogger().error(`Error processing string command: ${e.message}`);
        }
    });

    // 2. Specific Motor command (/motor_cmd)
    try {
        node.createSubscription('i2c_interfaces/msg/MotorCmd', '/motor_cmd', (msg) => {
            motor(msg.id.toUpperCase(), msg.direction, msg.intensity);
        });
        node.getLogger().info('Subscribed to /motor_cmd (i2c_interfaces/msg/MotorCmd)');
    } catch (e) {
        node.getLogger().warn('Could not subscribe to /motor_cmd: ' + e.message);
    }

    // 3. Specific DAC command (/dac_cmd)
    try {
        node.createSubscription('i2c_interfaces/msg/DacCmd', '/dac_cmd', (msg) => {
            const addr = msg.channel === 1 ? DAC1 : (msg.channel === 2 ? DAC2 : null);
            if (addr) dacSet(addr, msg.value);
            else node.getLogger().warn(`Invalid DAC channel: ${msg.channel}`);
        });
        node.getLogger().info('Subscribed to /dac_cmd (i2c_interfaces/msg/DacCmd)');
    } catch (e) {
        node.getLogger().warn('Could not subscribe to /dac_cmd: ' + e.message);
    }

    // 4. Specific Raw I2C command (/i2c_raw)
    try {
        node.createSubscription('i2c_interfaces/msg/I2cRaw', '/i2c_raw', (msg) => {
            if (!bus) return;
            bus.writeI2cBlockSync(msg.address, msg.reg, msg.data.length, Buffer.from(msg.data));
            node.getLogger().info(`Raw I2C write to 0x${msg.address.toString(16)}: [${msg.data}]`);
        });
        node.getLogger().info('Subscribed to /i2c_raw (i2c_interfaces/msg/I2cRaw)');
    } catch (e) {
        node.getLogger().warn('Could not subscribe to /i2c_raw: ' + e.message);
    }

    node.getLogger().info('I2C Bridge JS node started with multiple topics.');
    rclnodejs.spin(node);
}

main().catch(err => {
    console.error('Fatal Node error:', err);
});
