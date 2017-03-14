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

class startstop {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.startstop = null;
    }
    else {
      if (initObj.hasOwnProperty('startstop')) {
        this.startstop = initObj.startstop
      }
      else {
        this.startstop = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type startstop
    // Serialize message field [startstop]
    bufferOffset = _serializer.uint8(obj.startstop, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type startstop
    let len;
    let data = new startstop(null);
    // Deserialize message field [startstop]
    data.startstop = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a message object
    return 'rvss_workshop/startstop';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '221ff8f0d7a331524376c6ba7a726ca6';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8 startstop 
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new startstop(null);
    if (msg.startstop !== undefined) {
      resolved.startstop = msg.startstop;
    }
    else {
      resolved.startstop = 0
    }

    return resolved;
    }
};

module.exports = startstop;
