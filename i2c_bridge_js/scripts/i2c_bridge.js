#!/usr/bin/env node
process.env.NODE_PATH = '/home/user/ros2_ws/src/i2c_bridge_js/node_modules';
require('module').Module._initPaths();
const rclnodejs = require('rclnodejs');
const i2c = require('i2c-bus');

const STM32 = 0x3F;
const DAC1 = 0x60;
const DAC2 = 0x61;
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
        const buf = Buffer.from([
            (value >> 8) & 0x0F,
            value & 0xFF,
        ]);
        bus.i2cWriteSync(addr, buf.length, buf);
        node.getLogger().info(`DAC [0x${addr.toString(16).toUpperCase()}] set to ${value}`);
    }

    function motor(id, direction, intensity = 255) {
        if (!bus) return;
        const buf = Buffer.from([id.charCodeAt(0), direction, intensity]);
        bus.i2cWriteSync(STM32, buf.length, buf);
        node.getLogger().info(`Motor ${id} -> direction ${direction}, intensity ${intensity}`);
    }

    function syncMotors(leftDir, leftPwm, rightDir, rightPwm) {
        if (!bus) return;
        const SYNC_REG = 0x4D;
        const buf = Buffer.from([
            SYNC_REG,
            leftDir,
            leftPwm,
            rightDir,
            rightPwm,
        ]);
        bus.i2cWriteSync(STM32, buf.length, buf);
    }

    function speedToCommand(speed, maxSpeed, maxPwm, minPwm, deadband) {
        if (Math.abs(speed) < deadband) {
            return { dir: 0, pwm: 0 };
        }

        let pwm = Math.min(Math.max(Math.abs(speed) / maxSpeed * maxPwm, 0), maxPwm);
        pwm = Math.floor(pwm);

        if (pwm > 0 && pwm < minPwm) {
            pwm = minPwm;
        }

        return {
            dir: speed > 0 ? 1 : 2,
            pwm,
        };
    }

    // 1. String command (/i2c_cmd) - Legacy string parsing
    node.createSubscription('std_msgs/msg/String', '/i2c_cmd', (msg) => {
        const cmdText = msg.data.trim();
        try {
            const args = cmdText.split(/\s+/);
            const cmd = args[0].toLowerCase();
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
                    const buf = Buffer.concat([Buffer.from([reg]), Buffer.from(data)]);
                    bus.i2cWriteSync(addr, buf.length, buf);
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
            const buf = Buffer.concat([Buffer.from([msg.reg]), Buffer.from(msg.data)]);
            bus.i2cWriteSync(msg.address, buf.length, buf);
            node.getLogger().info(`Raw I2C write to 0x${msg.address.toString(16)}: [${msg.data}]`);
        });
        node.getLogger().info('Subscribed to /i2c_raw (i2c_interfaces/msg/I2cRaw)');
    } catch (e) {
        node.getLogger().warn('Could not subscribe to /i2c_raw: ' + e.message);
    }

    // 5. Binary Sync Motor command (/i2c_cmd_bin)
    node.createSubscription('std_msgs/msg/UInt8MultiArray', '/i2c_cmd_bin', (msg) => {
        const data = msg.data;
        if (data.length === 4) {
            syncMotors(data[0], data[1], data[2], data[3]);
        }
    });

    // 6. Navigation command (/cmd_vel)
    node.createSubscription('geometry_msgs/msg/Twist', '/cmd_vel', { qos: 'qos_profile_sensor_data' }, (msg) => {
        if (!bus) return;

        const v = msg.linear.x;
        const w = msg.angular.z;
        const wheelSep = 0.16;
        const maxSpeed = 0.5;
        const maxPwm = 255;
        const minPwm = 120;
        const deadband = 0.01;

        const leftSpeed = v - (w * wheelSep / 2.0);
        const rightSpeed = v + (w * wheelSep / 2.0);
        const left = speedToCommand(leftSpeed, maxSpeed, maxPwm, minPwm, deadband);
        const right = speedToCommand(rightSpeed, maxSpeed, maxPwm, minPwm, deadband);

        const bufA = Buffer.from([0x41, left.dir, left.pwm]);
        bus.i2cWriteSync(STM32, bufA.length, bufA);

        const bufB = Buffer.from([0x42, right.dir, right.pwm]);
        bus.i2cWriteSync(STM32, bufB.length, bufB);
    });

    node.getLogger().info('I2C Bridge JS node started with multiple topics (including /cmd_vel).');
    rclnodejs.spin(node);
}

main().catch(err => {
    console.error('Fatal Node error:', err);
});
