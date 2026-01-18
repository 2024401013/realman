// Auto-generated. Do not edit!

// (in-package rm_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Programrunstate {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.run_state = null;
      this.id = null;
      this.edit_id = null;
      this.plan_num = null;
      this.total_loop = null;
      this.step_mode = null;
      this.plan_speed = null;
      this.loop_num = null;
      this.loop_cont = null;
    }
    else {
      if (initObj.hasOwnProperty('run_state')) {
        this.run_state = initObj.run_state
      }
      else {
        this.run_state = 0;
      }
      if (initObj.hasOwnProperty('id')) {
        this.id = initObj.id
      }
      else {
        this.id = 0;
      }
      if (initObj.hasOwnProperty('edit_id')) {
        this.edit_id = initObj.edit_id
      }
      else {
        this.edit_id = 0;
      }
      if (initObj.hasOwnProperty('plan_num')) {
        this.plan_num = initObj.plan_num
      }
      else {
        this.plan_num = 0;
      }
      if (initObj.hasOwnProperty('total_loop')) {
        this.total_loop = initObj.total_loop
      }
      else {
        this.total_loop = 0;
      }
      if (initObj.hasOwnProperty('step_mode')) {
        this.step_mode = initObj.step_mode
      }
      else {
        this.step_mode = 0;
      }
      if (initObj.hasOwnProperty('plan_speed')) {
        this.plan_speed = initObj.plan_speed
      }
      else {
        this.plan_speed = 0;
      }
      if (initObj.hasOwnProperty('loop_num')) {
        this.loop_num = initObj.loop_num
      }
      else {
        this.loop_num = [];
      }
      if (initObj.hasOwnProperty('loop_cont')) {
        this.loop_cont = initObj.loop_cont
      }
      else {
        this.loop_cont = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Programrunstate
    // Serialize message field [run_state]
    bufferOffset = _serializer.int32(obj.run_state, buffer, bufferOffset);
    // Serialize message field [id]
    bufferOffset = _serializer.int32(obj.id, buffer, bufferOffset);
    // Serialize message field [edit_id]
    bufferOffset = _serializer.int32(obj.edit_id, buffer, bufferOffset);
    // Serialize message field [plan_num]
    bufferOffset = _serializer.int32(obj.plan_num, buffer, bufferOffset);
    // Serialize message field [total_loop]
    bufferOffset = _serializer.int32(obj.total_loop, buffer, bufferOffset);
    // Serialize message field [step_mode]
    bufferOffset = _serializer.int32(obj.step_mode, buffer, bufferOffset);
    // Serialize message field [plan_speed]
    bufferOffset = _serializer.int32(obj.plan_speed, buffer, bufferOffset);
    // Serialize message field [loop_num]
    bufferOffset = _arraySerializer.int32(obj.loop_num, buffer, bufferOffset, null);
    // Serialize message field [loop_cont]
    bufferOffset = _arraySerializer.int32(obj.loop_cont, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Programrunstate
    let len;
    let data = new Programrunstate(null);
    // Deserialize message field [run_state]
    data.run_state = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [id]
    data.id = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [edit_id]
    data.edit_id = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [plan_num]
    data.plan_num = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [total_loop]
    data.total_loop = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [step_mode]
    data.step_mode = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [plan_speed]
    data.plan_speed = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [loop_num]
    data.loop_num = _arrayDeserializer.int32(buffer, bufferOffset, null)
    // Deserialize message field [loop_cont]
    data.loop_cont = _arrayDeserializer.int32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.loop_num.length;
    length += 4 * object.loop_cont.length;
    return length + 36;
  }

  static datatype() {
    // Returns string type for a message object
    return 'rm_msgs/Programrunstate';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f985b60b566dc245fe01afe38654a230';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 run_state   # 运行状态 0 未开始 1运行中 2暂停中
    int32 id          # 运行轨迹编号
    int32 edit_id     # 上次编辑的在线编程编号 id
    int32 plan_num    # 运行行数
    int32 total_loop      # 循环指令数量
    int32 step_mode       # 单步模式，1 为单步模式，0 为非单步模式
    int32 plan_speed      # 全局规划速度比例 1-100
    int32[] loop_num        # 循环行数
    int32[] loop_cont       # 对应循环次数
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Programrunstate(null);
    if (msg.run_state !== undefined) {
      resolved.run_state = msg.run_state;
    }
    else {
      resolved.run_state = 0
    }

    if (msg.id !== undefined) {
      resolved.id = msg.id;
    }
    else {
      resolved.id = 0
    }

    if (msg.edit_id !== undefined) {
      resolved.edit_id = msg.edit_id;
    }
    else {
      resolved.edit_id = 0
    }

    if (msg.plan_num !== undefined) {
      resolved.plan_num = msg.plan_num;
    }
    else {
      resolved.plan_num = 0
    }

    if (msg.total_loop !== undefined) {
      resolved.total_loop = msg.total_loop;
    }
    else {
      resolved.total_loop = 0
    }

    if (msg.step_mode !== undefined) {
      resolved.step_mode = msg.step_mode;
    }
    else {
      resolved.step_mode = 0
    }

    if (msg.plan_speed !== undefined) {
      resolved.plan_speed = msg.plan_speed;
    }
    else {
      resolved.plan_speed = 0
    }

    if (msg.loop_num !== undefined) {
      resolved.loop_num = msg.loop_num;
    }
    else {
      resolved.loop_num = []
    }

    if (msg.loop_cont !== undefined) {
      resolved.loop_cont = msg.loop_cont;
    }
    else {
      resolved.loop_cont = []
    }

    return resolved;
    }
};

module.exports = Programrunstate;
