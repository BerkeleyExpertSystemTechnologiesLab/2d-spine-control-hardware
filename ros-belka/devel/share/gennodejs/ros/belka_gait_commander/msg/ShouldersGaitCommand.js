// Auto-generated. Do not edit!

// (in-package belka_gait_commander.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class ShouldersGaitCommand {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.shoulders_command = null;
    }
    else {
      if (initObj.hasOwnProperty('shoulders_command')) {
        this.shoulders_command = initObj.shoulders_command
      }
      else {
        this.shoulders_command = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ShouldersGaitCommand
    // Serialize message field [shoulders_command]
    bufferOffset = _serializer.string(obj.shoulders_command, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ShouldersGaitCommand
    let len;
    let data = new ShouldersGaitCommand(null);
    // Deserialize message field [shoulders_command]
    data.shoulders_command = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.shoulders_command.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'belka_gait_commander/ShouldersGaitCommand';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ba9308f196c49d06e4333c12eec0f361';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string shoulders_command
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ShouldersGaitCommand(null);
    if (msg.shoulders_command !== undefined) {
      resolved.shoulders_command = msg.shoulders_command;
    }
    else {
      resolved.shoulders_command = ''
    }

    return resolved;
    }
};

module.exports = ShouldersGaitCommand;
