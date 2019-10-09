// Auto-generated. Do not edit!

// (in-package belka-gait-commander.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class HipsGaitCommand {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.hips_command = null;
    }
    else {
      if (initObj.hasOwnProperty('hips_command')) {
        this.hips_command = initObj.hips_command
      }
      else {
        this.hips_command = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type HipsGaitCommand
    // Serialize message field [hips_command]
    bufferOffset = _serializer.string(obj.hips_command, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type HipsGaitCommand
    let len;
    let data = new HipsGaitCommand(null);
    // Deserialize message field [hips_command]
    data.hips_command = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.hips_command.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'belka-gait-commander/HipsGaitCommand';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3e35c60667a75681d37226fc0b36d0b6';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string hips_command
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new HipsGaitCommand(null);
    if (msg.hips_command !== undefined) {
      resolved.hips_command = msg.hips_command;
    }
    else {
      resolved.hips_command = ''
    }

    return resolved;
    }
};

module.exports = HipsGaitCommand;
