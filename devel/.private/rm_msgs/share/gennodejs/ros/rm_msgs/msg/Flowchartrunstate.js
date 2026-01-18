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

class Flowchartrunstate {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.run_state = null;
      this.id = null;
      this.name = null;
      this.plan_speed = null;
      this.step_mode = null;
      this.modal_id = null;
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
      if (initObj.hasOwnProperty('name')) {
        this.name = initObj.name
      }
      else {
        this.name = '';
      }
      if (initObj.hasOwnProperty('plan_speed')) {
        this.plan_speed = initObj.plan_speed
      }
      else {
        this.plan_speed = 0;
      }
      if (initObj.hasOwnProperty('step_mode')) {
        this.step_mode = initObj.step_mode
      }
      else {
        this.step_mode = 0;
      }
      if (initObj.hasOwnProperty('modal_id')) {
        this.modal_id = initObj.modal_id
      }
      else {
        this.modal_id = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Flowchartrunstate
    // Serialize message field [run_state]
    bufferOffset = _serializer.uint8(obj.run_state, buffer, bufferOffset);
    // Serialize message field [id]
    bufferOffset = _serializer.uint8(obj.id, buffer, bufferOffset);
    // Serialize message field [name]
    bufferOffset = _serializer.string(obj.name, buffer, bufferOffset);
    // Serialize message field [plan_speed]
    bufferOffset = _serializer.uint8(obj.plan_speed, buffer, bufferOffset);
    // Serialize message field [step_mode]
    bufferOffset = _serializer.uint8(obj.step_mode, buffer, bufferOffset);
    // Serialize message field [modal_id]
    bufferOffset = _serializer.string(obj.modal_id, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Flowchartrunstate
    let len;
    let data = new Flowchartrunstate(null);
    // Deserialize message field [run_state]
    data.run_state = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [id]
    data.id = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [name]
    data.name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [plan_speed]
    data.plan_speed = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [step_mode]
    data.step_mode = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [modal_id]
    data.modal_id = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.name);
    length += _getByteLength(object.modal_id);
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'rm_msgs/Flowchartrunstate';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '05f1a2e93a88cd1e5f9f80d8af8e987a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8 run_state  # 运行状态 0 未开始 1运行中 2暂停中
    uint8 id         # 当前使能的文件id。
    string name # 当前使能的文件名称。
    uint8 plan_speed     # 当前使能的文件全局规划速度比例 1-100。
    uint8 step_mode    # 单步模式，0为空，1为正常, 2为单步。
    string modal_id   # 运行到的流程图块的id。未运行则不返回
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Flowchartrunstate(null);
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

    if (msg.name !== undefined) {
      resolved.name = msg.name;
    }
    else {
      resolved.name = ''
    }

    if (msg.plan_speed !== undefined) {
      resolved.plan_speed = msg.plan_speed;
    }
    else {
      resolved.plan_speed = 0
    }

    if (msg.step_mode !== undefined) {
      resolved.step_mode = msg.step_mode;
    }
    else {
      resolved.step_mode = 0
    }

    if (msg.modal_id !== undefined) {
      resolved.modal_id = msg.modal_id;
    }
    else {
      resolved.modal_id = ''
    }

    return resolved;
    }
};

module.exports = Flowchartrunstate;
