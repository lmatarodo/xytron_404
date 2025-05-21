// Auto-generated. Do not edit!

// (in-package xycar_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class laneinfo {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.left_x = null;
      this.left_y = null;
      this.left_slope = null;
      this.right_x = null;
      this.right_y = null;
      this.right_slope = null;
    }
    else {
      if (initObj.hasOwnProperty('left_x')) {
        this.left_x = initObj.left_x
      }
      else {
        this.left_x = 0.0;
      }
      if (initObj.hasOwnProperty('left_y')) {
        this.left_y = initObj.left_y
      }
      else {
        this.left_y = 0.0;
      }
      if (initObj.hasOwnProperty('left_slope')) {
        this.left_slope = initObj.left_slope
      }
      else {
        this.left_slope = 0.0;
      }
      if (initObj.hasOwnProperty('right_x')) {
        this.right_x = initObj.right_x
      }
      else {
        this.right_x = 0.0;
      }
      if (initObj.hasOwnProperty('right_y')) {
        this.right_y = initObj.right_y
      }
      else {
        this.right_y = 0.0;
      }
      if (initObj.hasOwnProperty('right_slope')) {
        this.right_slope = initObj.right_slope
      }
      else {
        this.right_slope = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type laneinfo
    // Serialize message field [left_x]
    bufferOffset = _serializer.float32(obj.left_x, buffer, bufferOffset);
    // Serialize message field [left_y]
    bufferOffset = _serializer.float32(obj.left_y, buffer, bufferOffset);
    // Serialize message field [left_slope]
    bufferOffset = _serializer.float32(obj.left_slope, buffer, bufferOffset);
    // Serialize message field [right_x]
    bufferOffset = _serializer.float32(obj.right_x, buffer, bufferOffset);
    // Serialize message field [right_y]
    bufferOffset = _serializer.float32(obj.right_y, buffer, bufferOffset);
    // Serialize message field [right_slope]
    bufferOffset = _serializer.float32(obj.right_slope, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type laneinfo
    let len;
    let data = new laneinfo(null);
    // Deserialize message field [left_x]
    data.left_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [left_y]
    data.left_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [left_slope]
    data.left_slope = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [right_x]
    data.right_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [right_y]
    data.right_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [right_slope]
    data.right_slope = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 24;
  }

  static datatype() {
    // Returns string type for a message object
    return 'xycar_msgs/laneinfo';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '44a42243e734addbc7ddd0e00184e0ab';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 left_x
    float32 left_y
    float32 left_slope
    float32 right_x
    float32 right_y
    float32 right_slope 
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new laneinfo(null);
    if (msg.left_x !== undefined) {
      resolved.left_x = msg.left_x;
    }
    else {
      resolved.left_x = 0.0
    }

    if (msg.left_y !== undefined) {
      resolved.left_y = msg.left_y;
    }
    else {
      resolved.left_y = 0.0
    }

    if (msg.left_slope !== undefined) {
      resolved.left_slope = msg.left_slope;
    }
    else {
      resolved.left_slope = 0.0
    }

    if (msg.right_x !== undefined) {
      resolved.right_x = msg.right_x;
    }
    else {
      resolved.right_x = 0.0
    }

    if (msg.right_y !== undefined) {
      resolved.right_y = msg.right_y;
    }
    else {
      resolved.right_y = 0.0
    }

    if (msg.right_slope !== undefined) {
      resolved.right_slope = msg.right_slope;
    }
    else {
      resolved.right_slope = 0.0
    }

    return resolved;
    }
};

module.exports = laneinfo;
