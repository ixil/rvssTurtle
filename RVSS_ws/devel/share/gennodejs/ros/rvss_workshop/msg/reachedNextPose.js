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

class reachedNextPose {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.reachedNextPose = null;
    }
    else {
      if (initObj.hasOwnProperty('reachedNextPose')) {
        this.reachedNextPose = initObj.reachedNextPose
      }
      else {
        this.reachedNextPose = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type reachedNextPose
    // Serialize message field [reachedNextPose]
    bufferOffset = _serializer.uint8(obj.reachedNextPose, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type reachedNextPose
    let len;
    let data = new reachedNextPose(null);
    // Deserialize message field [reachedNextPose]
    data.reachedNextPose = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a message object
    return 'rvss_workshop/reachedNextPose';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '599100cb215b4875f186eebbd5850cec';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8 reachedNextPose
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new reachedNextPose(null);
    if (msg.reachedNextPose !== undefined) {
      resolved.reachedNextPose = msg.reachedNextPose;
    }
    else {
      resolved.reachedNextPose = 0
    }

    return resolved;
    }
};

module.exports = reachedNextPose;
