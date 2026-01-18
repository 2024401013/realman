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

class Start_Multi_Drag_Teach {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.free_axes = null;
      this.frame = null;
      this.singular_wall = null;
    }
    else {
      if (initObj.hasOwnProperty('free_axes')) {
        this.free_axes = initObj.free_axes
      }
      else {
        this.free_axes = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('frame')) {
        this.frame = initObj.frame
      }
      else {
        this.frame = 0;
      }
      if (initObj.hasOwnProperty('singular_wall')) {
        this.singular_wall = initObj.singular_wall
      }
      else {
        this.singular_wall = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Start_Multi_Drag_Teach
    // Check that the constant length array field [free_axes] has the right length
    if (obj.free_axes.length !== 6) {
      throw new Error('Unable to serialize array field free_axes - length must be 6')
    }
    // Serialize message field [free_axes]
    bufferOffset = _arraySerializer.int32(obj.free_axes, buffer, bufferOffset, 6);
    // Serialize message field [frame]
    bufferOffset = _serializer.int32(obj.frame, buffer, bufferOffset);
    // Serialize message field [singular_wall]
    bufferOffset = _serializer.int32(obj.singular_wall, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Start_Multi_Drag_Teach
    let len;
    let data = new Start_Multi_Drag_Teach(null);
    // Deserialize message field [free_axes]
    data.free_axes = _arrayDeserializer.int32(buffer, bufferOffset, 6)
    // Deserialize message field [frame]
    data.frame = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [singular_wall]
    data.singular_wall = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 32;
  }

  static datatype() {
    // Returns string type for a message object
    return 'rm_msgs/Start_Multi_Drag_Teach';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '36a7e83e1c20d27bf9d63d9ab11797e1';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #uint8 mode
    int32[6] free_axes       # 自由驱动方向[x,y,z,rx,ry,rz]，0-在参考坐标系对应方向轴上不可拖动，1-在参考坐标系对应方向轴上可拖动
    int32 frame              # 参考坐标系，0-工作坐标系 1-工具坐标系。
    int32 singular_wall      # 仅在六维力模式拖动示教中生效，用于指定是否开启拖动奇异墙，0表示关闭拖动奇异墙，1表示开启拖动奇异墙，若无配置参数，默认启动拖动奇异墙
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Start_Multi_Drag_Teach(null);
    if (msg.free_axes !== undefined) {
      resolved.free_axes = msg.free_axes;
    }
    else {
      resolved.free_axes = new Array(6).fill(0)
    }

    if (msg.frame !== undefined) {
      resolved.frame = msg.frame;
    }
    else {
      resolved.frame = 0
    }

    if (msg.singular_wall !== undefined) {
      resolved.singular_wall = msg.singular_wall;
    }
    else {
      resolved.singular_wall = 0
    }

    return resolved;
    }
};

module.exports = Start_Multi_Drag_Teach;
