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

class Hand_Seq {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.seq_num = null;
      this.block = null;
    }
    else {
      if (initObj.hasOwnProperty('seq_num')) {
        this.seq_num = initObj.seq_num
      }
      else {
        this.seq_num = 0;
      }
      if (initObj.hasOwnProperty('block')) {
        this.block = initObj.block
      }
      else {
        this.block = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Hand_Seq
    // Serialize message field [seq_num]
    bufferOffset = _serializer.uint16(obj.seq_num, buffer, bufferOffset);
    // Serialize message field [block]
    bufferOffset = _serializer.bool(obj.block, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Hand_Seq
    let len;
    let data = new Hand_Seq(null);
    // Deserialize message field [seq_num]
    data.seq_num = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [block]
    data.block = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 3;
  }

  static datatype() {
    // Returns string type for a message object
    return 'rm_msgs/Hand_Seq';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f00b164e44237e7253d6d77c16c73f25';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #设置灵巧手动作序列
    uint16 seq_num         #预先保存在灵巧手内的序列序号，范围：1~40
    bool block #true 表示阻塞模式，false 表示非阻塞模式。
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Hand_Seq(null);
    if (msg.seq_num !== undefined) {
      resolved.seq_num = msg.seq_num;
    }
    else {
      resolved.seq_num = 0
    }

    if (msg.block !== undefined) {
      resolved.block = msg.block;
    }
    else {
      resolved.block = false
    }

    return resolved;
    }
};

module.exports = Hand_Seq;
