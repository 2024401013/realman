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

class ExpandState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.get_state = null;
      this.pos = null;
      this.err_flag = null;
      this.en_flag = null;
      this.current = null;
      this.mode = null;
      this.joint_id = null;
    }
    else {
      if (initObj.hasOwnProperty('get_state')) {
        this.get_state = initObj.get_state
      }
      else {
        this.get_state = false;
      }
      if (initObj.hasOwnProperty('pos')) {
        this.pos = initObj.pos
      }
      else {
        this.pos = 0;
      }
      if (initObj.hasOwnProperty('err_flag')) {
        this.err_flag = initObj.err_flag
      }
      else {
        this.err_flag = 0;
      }
      if (initObj.hasOwnProperty('en_flag')) {
        this.en_flag = initObj.en_flag
      }
      else {
        this.en_flag = 0;
      }
      if (initObj.hasOwnProperty('current')) {
        this.current = initObj.current
      }
      else {
        this.current = 0;
      }
      if (initObj.hasOwnProperty('mode')) {
        this.mode = initObj.mode
      }
      else {
        this.mode = 0;
      }
      if (initObj.hasOwnProperty('joint_id')) {
        this.joint_id = initObj.joint_id
      }
      else {
        this.joint_id = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ExpandState
    // Serialize message field [get_state]
    bufferOffset = _serializer.bool(obj.get_state, buffer, bufferOffset);
    // Serialize message field [pos]
    bufferOffset = _serializer.int16(obj.pos, buffer, bufferOffset);
    // Serialize message field [err_flag]
    bufferOffset = _serializer.int16(obj.err_flag, buffer, bufferOffset);
    // Serialize message field [en_flag]
    bufferOffset = _serializer.uint16(obj.en_flag, buffer, bufferOffset);
    // Serialize message field [current]
    bufferOffset = _serializer.uint16(obj.current, buffer, bufferOffset);
    // Serialize message field [mode]
    bufferOffset = _serializer.byte(obj.mode, buffer, bufferOffset);
    // Serialize message field [joint_id]
    bufferOffset = _serializer.uint16(obj.joint_id, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ExpandState
    let len;
    let data = new ExpandState(null);
    // Deserialize message field [get_state]
    data.get_state = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [pos]
    data.pos = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [err_flag]
    data.err_flag = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [en_flag]
    data.en_flag = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [current]
    data.current = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [mode]
    data.mode = _deserializer.byte(buffer, bufferOffset);
    // Deserialize message field [joint_id]
    data.joint_id = _deserializer.uint16(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'rm_msgs/ExpandState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '51186bb81aaa7ad5c549fa4631da2340';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool get_state
    int16 pos	    #扩展关节角度，单位度，精度 0.001°
    int16 err_flag	#驱动错误代码
    uint16 en_flag	#扩展关节使能状态
    uint16 current	#当前驱动电流，单位：mA，精度：1mA。
    byte mode	    #当前扩展关节状态，0-空闲，1-正方向速度运动，2-正方向位置运动，3-负方向速度运动，4-负方向位置运动
    uint16 joint_id	#扩展关节ID
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ExpandState(null);
    if (msg.get_state !== undefined) {
      resolved.get_state = msg.get_state;
    }
    else {
      resolved.get_state = false
    }

    if (msg.pos !== undefined) {
      resolved.pos = msg.pos;
    }
    else {
      resolved.pos = 0
    }

    if (msg.err_flag !== undefined) {
      resolved.err_flag = msg.err_flag;
    }
    else {
      resolved.err_flag = 0
    }

    if (msg.en_flag !== undefined) {
      resolved.en_flag = msg.en_flag;
    }
    else {
      resolved.en_flag = 0
    }

    if (msg.current !== undefined) {
      resolved.current = msg.current;
    }
    else {
      resolved.current = 0
    }

    if (msg.mode !== undefined) {
      resolved.mode = msg.mode;
    }
    else {
      resolved.mode = 0
    }

    if (msg.joint_id !== undefined) {
      resolved.joint_id = msg.joint_id;
    }
    else {
      resolved.joint_id = 0
    }

    return resolved;
    }
};

module.exports = ExpandState;
