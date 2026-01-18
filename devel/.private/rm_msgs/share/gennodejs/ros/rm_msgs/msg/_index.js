
"use strict";

let Arm_Current_State = require('./Arm_Current_State.js');
let ChangeTool_State = require('./ChangeTool_State.js');
let Get_TCP_Master_List_Param = require('./Get_TCP_Master_List_Param.js');
let Force_Position_State = require('./Force_Position_State.js');
let Force_Position_Move_Joint = require('./Force_Position_Move_Joint.js');
let UpdateTCPmasterparam = require('./UpdateTCPmasterparam.js');
let Arm_Softversion_v4 = require('./Arm_Softversion_v4.js');
let Hand_Angle = require('./Hand_Angle.js');
let Hand_Force = require('./Hand_Force.js');
let CartePosCustom = require('./CartePosCustom.js');
let Write_ModbusRTU = require('./Write_ModbusRTU.js');
let Tool_Digital_Output = require('./Tool_Digital_Output.js');
let Joint_Max_Speed = require('./Joint_Max_Speed.js');
let Modbustcpmasterlist = require('./Modbustcpmasterlist.js');
let Expand_Position = require('./Expand_Position.js');
let Arm_Joint_Speed_Max = require('./Arm_Joint_Speed_Max.js');
let Moveloffset = require('./Moveloffset.js');
let Lift_Speed = require('./Lift_Speed.js');
let JointPos = require('./JointPos.js');
let Register_Data = require('./Register_Data.js');
let Ort_Teach = require('./Ort_Teach.js');
let Stop = require('./Stop.js');
let Set_Force_Position = require('./Set_Force_Position.js');
let Tool_IO_State = require('./Tool_IO_State.js');
let Joint_Enable = require('./Joint_Enable.js');
let Write_TCPandRTU = require('./Write_TCPandRTU.js');
let Manual_Set_Force_Pose = require('./Manual_Set_Force_Pose.js');
let Force_Position_Move_Pose = require('./Force_Position_Move_Pose.js');
let Arm_IO_State = require('./Arm_IO_State.js');
let Arm_Softversion_v3 = require('./Arm_Softversion_v3.js');
let Plan_State = require('./Plan_State.js');
let Programrunstate = require('./Programrunstate.js');
let Joint_En_Flag = require('./Joint_En_Flag.js');
let ChangeWorkFrame_State = require('./ChangeWorkFrame_State.js');
let Socket_Command = require('./Socket_Command.js');
let Read_TCPandRTU = require('./Read_TCPandRTU.js');
let Joint_Current = require('./Joint_Current.js');
let Jointversion = require('./Jointversion.js');
let Joint_Temperature = require('./Joint_Temperature.js');
let CartePos = require('./CartePos.js');
let RS485_Mode = require('./RS485_Mode.js');
let Expand_In_Position = require('./Expand_In_Position.js');
let Hand_Status = require('./Hand_Status.js');
let RS485params = require('./RS485params.js');
let Gripper_Pick = require('./Gripper_Pick.js');
let Modbustcpmasterinfo = require('./Modbustcpmasterinfo.js');
let Servo_GetAngle = require('./Servo_GetAngle.js');
let MoveJ = require('./MoveJ.js');
let Gettrajectorylist = require('./Gettrajectorylist.js');
let Joint_Speed = require('./Joint_Speed.js');
let Joint_PoseEuler = require('./Joint_PoseEuler.js');
let Set_Modbus_Mode = require('./Set_Modbus_Mode.js');
let ArmState = require('./ArmState.js');
let MoveL = require('./MoveL.js');
let Rm_Plus_State = require('./Rm_Plus_State.js');
let MoveC = require('./MoveC.js');
let Write_Register = require('./Write_Register.js');
let Hand_Seq = require('./Hand_Seq.js');
let ChangeWorkFrame_Name = require('./ChangeWorkFrame_Name.js');
let Turtle_Driver = require('./Turtle_Driver.js');
let Arm_Digital_Output = require('./Arm_Digital_Output.js');
let Gripper_Set = require('./Gripper_Set.js');
let Softwarebuildinfo = require('./Softwarebuildinfo.js');
let Joint_Step = require('./Joint_Step.js');
let Rm_Plus_Base = require('./Rm_Plus_Base.js');
let MoveJ_P = require('./MoveJ_P.js');
let Servo_Move = require('./Servo_Move.js');
let Pos_Teach = require('./Pos_Teach.js');
let Trajectorylist = require('./Trajectorylist.js');
let Tool_Analog_Output = require('./Tool_Analog_Output.js');
let JointPosCustom = require('./JointPosCustom.js');
let Trajectoryinfo = require('./Trajectoryinfo.js');
let Modbusreaddata = require('./Modbusreaddata.js');
let GetArmState_Command = require('./GetArmState_Command.js');
let Six_Force = require('./Six_Force.js');
let Arm_Analog_Output = require('./Arm_Analog_Output.js');
let Lift_In_Position = require('./Lift_In_Position.js');
let Mastername = require('./Mastername.js');
let Read_Register = require('./Read_Register.js');
let Flowchartrunstate = require('./Flowchartrunstate.js');
let ExpandState = require('./ExpandState.js');
let Expand_Speed = require('./Expand_Speed.js');
let Hand_Speed = require('./Hand_Speed.js');
let Read_ModbusRTU = require('./Read_ModbusRTU.js');
let Joint_Error_Code = require('./Joint_Error_Code.js');
let Cabinet = require('./Cabinet.js');
let Start_Multi_Drag_Teach = require('./Start_Multi_Drag_Teach.js');
let Joint_Voltage = require('./Joint_Voltage.js');
let Hand_Posture = require('./Hand_Posture.js');
let Arm_Software_Version = require('./Arm_Software_Version.js');
let Arm_Current_Status = require('./Arm_Current_Status.js');
let IO_Update = require('./IO_Update.js');
let Stop_Teach = require('./Stop_Teach.js');
let Set_Realtime_Push = require('./Set_Realtime_Push.js');
let LiftState = require('./LiftState.js');
let Joint_Teach = require('./Joint_Teach.js');
let Err = require('./Err.js');
let Force_Position_Move_Pose_Custom = require('./Force_Position_Move_Pose_Custom.js');
let Lift_Height = require('./Lift_Height.js');
let ChangeTool_Name = require('./ChangeTool_Name.js');

module.exports = {
  Arm_Current_State: Arm_Current_State,
  ChangeTool_State: ChangeTool_State,
  Get_TCP_Master_List_Param: Get_TCP_Master_List_Param,
  Force_Position_State: Force_Position_State,
  Force_Position_Move_Joint: Force_Position_Move_Joint,
  UpdateTCPmasterparam: UpdateTCPmasterparam,
  Arm_Softversion_v4: Arm_Softversion_v4,
  Hand_Angle: Hand_Angle,
  Hand_Force: Hand_Force,
  CartePosCustom: CartePosCustom,
  Write_ModbusRTU: Write_ModbusRTU,
  Tool_Digital_Output: Tool_Digital_Output,
  Joint_Max_Speed: Joint_Max_Speed,
  Modbustcpmasterlist: Modbustcpmasterlist,
  Expand_Position: Expand_Position,
  Arm_Joint_Speed_Max: Arm_Joint_Speed_Max,
  Moveloffset: Moveloffset,
  Lift_Speed: Lift_Speed,
  JointPos: JointPos,
  Register_Data: Register_Data,
  Ort_Teach: Ort_Teach,
  Stop: Stop,
  Set_Force_Position: Set_Force_Position,
  Tool_IO_State: Tool_IO_State,
  Joint_Enable: Joint_Enable,
  Write_TCPandRTU: Write_TCPandRTU,
  Manual_Set_Force_Pose: Manual_Set_Force_Pose,
  Force_Position_Move_Pose: Force_Position_Move_Pose,
  Arm_IO_State: Arm_IO_State,
  Arm_Softversion_v3: Arm_Softversion_v3,
  Plan_State: Plan_State,
  Programrunstate: Programrunstate,
  Joint_En_Flag: Joint_En_Flag,
  ChangeWorkFrame_State: ChangeWorkFrame_State,
  Socket_Command: Socket_Command,
  Read_TCPandRTU: Read_TCPandRTU,
  Joint_Current: Joint_Current,
  Jointversion: Jointversion,
  Joint_Temperature: Joint_Temperature,
  CartePos: CartePos,
  RS485_Mode: RS485_Mode,
  Expand_In_Position: Expand_In_Position,
  Hand_Status: Hand_Status,
  RS485params: RS485params,
  Gripper_Pick: Gripper_Pick,
  Modbustcpmasterinfo: Modbustcpmasterinfo,
  Servo_GetAngle: Servo_GetAngle,
  MoveJ: MoveJ,
  Gettrajectorylist: Gettrajectorylist,
  Joint_Speed: Joint_Speed,
  Joint_PoseEuler: Joint_PoseEuler,
  Set_Modbus_Mode: Set_Modbus_Mode,
  ArmState: ArmState,
  MoveL: MoveL,
  Rm_Plus_State: Rm_Plus_State,
  MoveC: MoveC,
  Write_Register: Write_Register,
  Hand_Seq: Hand_Seq,
  ChangeWorkFrame_Name: ChangeWorkFrame_Name,
  Turtle_Driver: Turtle_Driver,
  Arm_Digital_Output: Arm_Digital_Output,
  Gripper_Set: Gripper_Set,
  Softwarebuildinfo: Softwarebuildinfo,
  Joint_Step: Joint_Step,
  Rm_Plus_Base: Rm_Plus_Base,
  MoveJ_P: MoveJ_P,
  Servo_Move: Servo_Move,
  Pos_Teach: Pos_Teach,
  Trajectorylist: Trajectorylist,
  Tool_Analog_Output: Tool_Analog_Output,
  JointPosCustom: JointPosCustom,
  Trajectoryinfo: Trajectoryinfo,
  Modbusreaddata: Modbusreaddata,
  GetArmState_Command: GetArmState_Command,
  Six_Force: Six_Force,
  Arm_Analog_Output: Arm_Analog_Output,
  Lift_In_Position: Lift_In_Position,
  Mastername: Mastername,
  Read_Register: Read_Register,
  Flowchartrunstate: Flowchartrunstate,
  ExpandState: ExpandState,
  Expand_Speed: Expand_Speed,
  Hand_Speed: Hand_Speed,
  Read_ModbusRTU: Read_ModbusRTU,
  Joint_Error_Code: Joint_Error_Code,
  Cabinet: Cabinet,
  Start_Multi_Drag_Teach: Start_Multi_Drag_Teach,
  Joint_Voltage: Joint_Voltage,
  Hand_Posture: Hand_Posture,
  Arm_Software_Version: Arm_Software_Version,
  Arm_Current_Status: Arm_Current_Status,
  IO_Update: IO_Update,
  Stop_Teach: Stop_Teach,
  Set_Realtime_Push: Set_Realtime_Push,
  LiftState: LiftState,
  Joint_Teach: Joint_Teach,
  Err: Err,
  Force_Position_Move_Pose_Custom: Force_Position_Move_Pose_Custom,
  Lift_Height: Lift_Height,
  ChangeTool_Name: ChangeTool_Name,
};
