// Auto-generated. Do not edit!

// (in-package rvss_workshop.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let cylMsg = require('./cylMsg.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class cylDataArray {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.cylinders = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('cylinders')) {
        this.cylinders = initObj.cylinders
      }
      else {
        this.cylinders = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type cylDataArray
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [cylinders]
    // Serialize the length for message field [cylinders]
    bufferOffset = _serializer.uint32(obj.cylinders.length, buffer, bufferOffset);
    obj.cylinders.forEach((val) => {
      bufferOffset = cylMsg.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type cylDataArray
    let len;
    let data = new cylDataArray(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [cylinders]
    // Deserialize array length for message field [cylinders]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.cylinders = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.cylinders[i] = cylMsg.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    object.cylinders.forEach((val) => {
      length += cylMsg.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'rvss_workshop/cylDataArray';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '59182e50b0c2a27f7f6f0f07e63a5e57';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    cylMsg[] cylinders
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    # 0: no frame
    # 1: global frame
    string frame_id
    
    ================================================================================
    MSG: rvss_workshop/cylMsg
    Header header
    float32 Zrobot
    float32 Xrobot
    uint8 label
    float32[] covariance
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new cylDataArray(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.cylinders !== undefined) {
      resolved.cylinders = new Array(msg.cylinders.length);
      for (let i = 0; i < resolved.cylinders.length; ++i) {
        resolved.cylinders[i] = cylMsg.Resolve(msg.cylinders[i]);
      }
    }
    else {
      resolved.cylinders = []
    }

    return resolved;
    }
};

module.exports = cylDataArray;
