// Auto-generated. Do not edit!

// (in-package rm_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Trajectoryinfo = require('./Trajectoryinfo.js');

//-----------------------------------------------------------

class Trajectorylist {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.page_num = null;
      this.page_size = null;
      this.total_size = null;
      this.vague_search = null;
      this.tra_list = null;
    }
    else {
      if (initObj.hasOwnProperty('page_num')) {
        this.page_num = initObj.page_num
      }
      else {
        this.page_num = 0;
      }
      if (initObj.hasOwnProperty('page_size')) {
        this.page_size = initObj.page_size
      }
      else {
        this.page_size = 0;
      }
      if (initObj.hasOwnProperty('total_size')) {
        this.total_size = initObj.total_size
      }
      else {
        this.total_size = 0;
      }
      if (initObj.hasOwnProperty('vague_search')) {
        this.vague_search = initObj.vague_search
      }
      else {
        this.vague_search = '';
      }
      if (initObj.hasOwnProperty('tra_list')) {
        this.tra_list = initObj.tra_list
      }
      else {
        this.tra_list = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Trajectorylist
    // Serialize message field [page_num]
    bufferOffset = _serializer.int32(obj.page_num, buffer, bufferOffset);
    // Serialize message field [page_size]
    bufferOffset = _serializer.int32(obj.page_size, buffer, bufferOffset);
    // Serialize message field [total_size]
    bufferOffset = _serializer.int32(obj.total_size, buffer, bufferOffset);
    // Serialize message field [vague_search]
    bufferOffset = _serializer.string(obj.vague_search, buffer, bufferOffset);
    // Serialize message field [tra_list]
    // Serialize the length for message field [tra_list]
    bufferOffset = _serializer.uint32(obj.tra_list.length, buffer, bufferOffset);
    obj.tra_list.forEach((val) => {
      bufferOffset = Trajectoryinfo.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Trajectorylist
    let len;
    let data = new Trajectorylist(null);
    // Deserialize message field [page_num]
    data.page_num = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [page_size]
    data.page_size = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [total_size]
    data.total_size = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [vague_search]
    data.vague_search = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [tra_list]
    // Deserialize array length for message field [tra_list]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.tra_list = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.tra_list[i] = Trajectoryinfo.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.vague_search);
    object.tra_list.forEach((val) => {
      length += Trajectoryinfo.getMessageSize(val);
    });
    return length + 20;
  }

  static datatype() {
    // Returns string type for a message object
    return 'rm_msgs/Trajectorylist';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '8ffd0485c7441f6956165c1adeb616bc';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 page_num      # 页码
    int32 page_size     # 每页大小
    int32 total_size    # 列表长度
    string vague_search  # 模糊搜索 
    Trajectoryinfo[] tra_list  # 返回符合的轨迹列表
    
    ================================================================================
    MSG: rm_msgs/Trajectoryinfo
    string name          #轨迹名称	
    string create_time   #创建时间
    int32 point_num      #轨迹点数量
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Trajectorylist(null);
    if (msg.page_num !== undefined) {
      resolved.page_num = msg.page_num;
    }
    else {
      resolved.page_num = 0
    }

    if (msg.page_size !== undefined) {
      resolved.page_size = msg.page_size;
    }
    else {
      resolved.page_size = 0
    }

    if (msg.total_size !== undefined) {
      resolved.total_size = msg.total_size;
    }
    else {
      resolved.total_size = 0
    }

    if (msg.vague_search !== undefined) {
      resolved.vague_search = msg.vague_search;
    }
    else {
      resolved.vague_search = ''
    }

    if (msg.tra_list !== undefined) {
      resolved.tra_list = new Array(msg.tra_list.length);
      for (let i = 0; i < resolved.tra_list.length; ++i) {
        resolved.tra_list[i] = Trajectoryinfo.Resolve(msg.tra_list[i]);
      }
    }
    else {
      resolved.tra_list = []
    }

    return resolved;
    }
};

module.exports = Trajectorylist;
