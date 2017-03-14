// Auto-generated. Do not edit!

// (in-package rvss_workshop.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class nextPose {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.nextPose = null;
    }
    else {
      if (initObj.hasOwnProperty('nextPose')) {
        this.nextPose = initObj.nextPose
      }
      else {
        this.nextPose = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type nextPose
    // Serialize message field [nextPose]
    bufferOffset = _arraySerializer.float32(obj.nextPose, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type nextPose
    let len;
    let data = new nextPose(null);
    // Deserialize message field [nextPose]
    data.nextPose = _arrayDeserializer.float32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.nextPose.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'rvss_workshop/nextPose';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '472aa9e9259455af5554754a873ea739';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32[] nextPose
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new nextPose(null);
    if (msg.nextPose !== undefined) {
      resolved.nextPose = msg.nextPose;
    }
    else {
      resolved.nextPose = []
    }

    return resolved;
    }
};

module.exports = nextPose;
