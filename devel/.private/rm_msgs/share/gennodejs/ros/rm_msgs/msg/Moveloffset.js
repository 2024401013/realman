// Auto-generated. Do not edit!

// (in-package rm_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class Moveloffset {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.offset = null;
      this.speed = null;
      this.r = null;
      this.trajectory_connect = null;
      this.frame_type = null;
      this.block = null;
    }
    else {
      if (initObj.hasOwnProperty('offset')) {
        this.offset = initObj.offset
      }
      else {
        this.offset = new geometry_msgs.msg.Pose();
      }
      if (initObj.hasOwnProperty('speed')) {
        this.speed = initObj.speed
      }
      else {
        this.speed = 0;
      }
      if (initObj.hasOwnProperty('r')) {
        this.r = initObj.r
      }
      else {
        this.r = 0;
      }
      if (initObj.hasOwnProperty('trajectory_connect')) {
        this.trajectory_connect = initObj.trajectory_connect
      }
      else {
        this.trajectory_connect = false;
      }
      if (initObj.hasOwnProperty('frame_type')) {
        this.frame_type = initObj.frame_type
      }
      else {
        this.frame_type = false;
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
    // Serializes a message object of type Moveloffset
    // Serialize message field [offset]
    bufferOffset = geometry_msgs.msg.Pose.serialize(obj.offset, buffer, bufferOffset);
    // Serialize message field [speed]
    bufferOffset = _serializer.int32(obj.speed, buffer, bufferOffset);
    // Serialize message field [r]
    bufferOffset = _serializer.int32(obj.r, buffer, bufferOffset);
    // Serialize message field [trajectory_connect]
    bufferOffset = _serializer.bool(obj.trajectory_connect, buffer, bufferOffset);
    // Serialize message field [frame_type]
    bufferOffset = _serializer.bool(obj.frame_type, buffer, bufferOffset);
    // Serialize message field [block]
    bufferOffset = _serializer.bool(obj.block, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Moveloffset
    let len;
    let data = new Moveloffset(null);
    // Deserialize message field [offset]
    data.offset = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset);
    // Deserialize message field [speed]
    data.speed = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [r]
    data.r = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [trajectory_connect]
    data.trajectory_connect = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [frame_type]
    data.frame_type = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [block]
    data.block = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 67;
  }

  static datatype() {
    // Returns string type for a message object
    return 'rm_msgs/Moveloffset';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'af2efa1aef4c53564271cdf8a17ca144';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    geometry_msgs/Pose offset
    int32 speed
    int32 r
    bool trajectory_connect
    bool frame_type
    bool block
    ================================================================================
    MSG: geometry_msgs/Pose
    # A representation of pose in free space, composed of position and orientation. 
    Point position
    Quaternion orientation
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Moveloffset(null);
    if (msg.offset !== undefined) {
      resolved.offset = geometry_msgs.msg.Pose.Resolve(msg.offset)
    }
    else {
      resolved.offset = new geometry_msgs.msg.Pose()
    }

    if (msg.speed !== undefined) {
      resolved.speed = msg.speed;
    }
    else {
      resolved.speed = 0
    }

    if (msg.r !== undefined) {
      resolved.r = msg.r;
    }
    else {
      resolved.r = 0
    }

    if (msg.trajectory_connect !== undefined) {
      resolved.trajectory_connect = msg.trajectory_connect;
    }
    else {
      resolved.trajectory_connect = false
    }

    if (msg.frame_type !== undefined) {
      resolved.frame_type = msg.frame_type;
    }
    else {
      resolved.frame_type = false
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

module.exports = Moveloffset;
