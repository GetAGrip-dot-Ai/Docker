
"use strict";

let ApiOptions = require('./ApiOptions.js');
let KortexError = require('./KortexError.js');
let ErrorCodes = require('./ErrorCodes.js');
let SubErrorCodes = require('./SubErrorCodes.js');
let ActuatorConfig_SafetyLimitType = require('./ActuatorConfig_SafetyLimitType.js');
let CommandMode = require('./CommandMode.js');
let ActuatorConfig_ServiceVersion = require('./ActuatorConfig_ServiceVersion.js');
let CustomDataIndex = require('./CustomDataIndex.js');
let CustomDataSelection = require('./CustomDataSelection.js');
let AxisOffsets = require('./AxisOffsets.js');
let ControlLoop = require('./ControlLoop.js');
let EncoderDerivativeParameters = require('./EncoderDerivativeParameters.js');
let TorqueCalibration = require('./TorqueCalibration.js');
let StepResponse = require('./StepResponse.js');
let VectorDriveParameters = require('./VectorDriveParameters.js');
let CommandModeInformation = require('./CommandModeInformation.js');
let Servoing = require('./Servoing.js');
let ControlLoopSelection = require('./ControlLoopSelection.js');
let TorqueOffset = require('./TorqueOffset.js');
let RampResponse = require('./RampResponse.js');
let PositionCommand = require('./PositionCommand.js');
let AxisPosition = require('./AxisPosition.js');
let ActuatorConfig_ControlMode = require('./ActuatorConfig_ControlMode.js');
let LoopSelection = require('./LoopSelection.js');
let FrequencyResponse = require('./FrequencyResponse.js');
let CoggingFeedforwardMode = require('./CoggingFeedforwardMode.js');
let CoggingFeedforwardModeInformation = require('./CoggingFeedforwardModeInformation.js');
let ControlLoopParameters = require('./ControlLoopParameters.js');
let SafetyIdentifierBankA = require('./SafetyIdentifierBankA.js');
let ActuatorConfig_ControlModeInformation = require('./ActuatorConfig_ControlModeInformation.js');
let StatusFlags = require('./StatusFlags.js');
let ActuatorCyclic_MessageId = require('./ActuatorCyclic_MessageId.js');
let ActuatorCyclic_Command = require('./ActuatorCyclic_Command.js');
let ActuatorCyclic_Feedback = require('./ActuatorCyclic_Feedback.js');
let ActuatorCyclic_CustomData = require('./ActuatorCyclic_CustomData.js');
let CommandFlags = require('./CommandFlags.js');
let ActuatorCyclic_ServiceVersion = require('./ActuatorCyclic_ServiceVersion.js');
let SafetyEvent = require('./SafetyEvent.js');
let FactoryEvent = require('./FactoryEvent.js');
let ControllerElementHandle = require('./ControllerElementHandle.js');
let UserNotificationList = require('./UserNotificationList.js');
let JointTorque = require('./JointTorque.js');
let ActionNotificationList = require('./ActionNotificationList.js');
let ControllerNotification_state = require('./ControllerNotification_state.js');
let ControllerElementEventType = require('./ControllerElementEventType.js');
let MapHandle = require('./MapHandle.js');
let ServoingMode = require('./ServoingMode.js');
let Orientation = require('./Orientation.js');
let ActuatorInformation = require('./ActuatorInformation.js');
let JointAngles = require('./JointAngles.js');
let GpioEvent = require('./GpioEvent.js');
let JointTorques = require('./JointTorques.js');
let ActionExecutionState = require('./ActionExecutionState.js');
let PreComputedJointTrajectoryElement = require('./PreComputedJointTrajectoryElement.js');
let OperatingMode = require('./OperatingMode.js');
let Ssid = require('./Ssid.js');
let ControllerType = require('./ControllerType.js');
let Base_CapSenseMode = require('./Base_CapSenseMode.js');
let MapEvent_events = require('./MapEvent_events.js');
let TwistCommand = require('./TwistCommand.js');
let BridgeType = require('./BridgeType.js');
let Wrench = require('./Wrench.js');
let ControllerConfigurationList = require('./ControllerConfigurationList.js');
let JointTrajectoryConstraintType = require('./JointTrajectoryConstraintType.js');
let WifiInformationList = require('./WifiInformationList.js');
let Base_RotationMatrix = require('./Base_RotationMatrix.js');
let ArmStateNotification = require('./ArmStateNotification.js');
let MapElement = require('./MapElement.js');
let ControllerNotification = require('./ControllerNotification.js');
let BridgePortConfig = require('./BridgePortConfig.js');
let SequenceTaskConfiguration = require('./SequenceTaskConfiguration.js');
let SequenceInfoNotificationList = require('./SequenceInfoNotificationList.js');
let CartesianTrajectoryConstraint = require('./CartesianTrajectoryConstraint.js');
let ActionHandle = require('./ActionHandle.js');
let ServoingModeNotification = require('./ServoingModeNotification.js');
let WrenchLimitation = require('./WrenchLimitation.js');
let ZoneShape = require('./ZoneShape.js');
let NetworkEvent = require('./NetworkEvent.js');
let JointSpeed = require('./JointSpeed.js');
let Base_RotationMatrixRow = require('./Base_RotationMatrixRow.js');
let FirmwareComponentVersion = require('./FirmwareComponentVersion.js');
let MapGroup = require('./MapGroup.js');
let ProtectionZoneNotification = require('./ProtectionZoneNotification.js');
let ConfigurationNotificationEvent = require('./ConfigurationNotificationEvent.js');
let LedState = require('./LedState.js');
let SequenceTask = require('./SequenceTask.js');
let PreComputedJointTrajectory = require('./PreComputedJointTrajectory.js');
let BridgeIdentifier = require('./BridgeIdentifier.js');
let WristDigitalInputIdentifier = require('./WristDigitalInputIdentifier.js');
let ConstrainedOrientation = require('./ConstrainedOrientation.js');
let OperatingModeNotificationList = require('./OperatingModeNotificationList.js');
let CartesianWaypoint = require('./CartesianWaypoint.js');
let BridgeStatus = require('./BridgeStatus.js');
let SequenceTasksRange = require('./SequenceTasksRange.js');
let Faults = require('./Faults.js');
let ProtectionZoneInformation = require('./ProtectionZoneInformation.js');
let KinematicTrajectoryConstraints = require('./KinematicTrajectoryConstraints.js');
let Base_ControlModeInformation = require('./Base_ControlModeInformation.js');
let ControllerNotificationList = require('./ControllerNotificationList.js');
let LimitationType = require('./LimitationType.js');
let UserNotification = require('./UserNotification.js');
let Base_JointSpeeds = require('./Base_JointSpeeds.js');
let ProtectionZone = require('./ProtectionZone.js');
let CartesianLimitationList = require('./CartesianLimitationList.js');
let UserProfileList = require('./UserProfileList.js');
let CartesianTrajectoryConstraint_type = require('./CartesianTrajectoryConstraint_type.js');
let UserProfile = require('./UserProfile.js');
let JointAngle = require('./JointAngle.js');
let ChangeWrench = require('./ChangeWrench.js');
let ControllerBehavior = require('./ControllerBehavior.js');
let Action_action_parameters = require('./Action_action_parameters.js');
let Point = require('./Point.js');
let Base_CapSenseConfig = require('./Base_CapSenseConfig.js');
let FactoryNotification = require('./FactoryNotification.js');
let PasswordChange = require('./PasswordChange.js');
let ControlModeNotificationList = require('./ControlModeNotificationList.js');
let TransformationRow = require('./TransformationRow.js');
let ControllerConfigurationMode = require('./ControllerConfigurationMode.js');
let CartesianSpeed = require('./CartesianSpeed.js');
let GripperCommand = require('./GripperCommand.js');
let JointsLimitationsList = require('./JointsLimitationsList.js');
let GpioPinConfiguration = require('./GpioPinConfiguration.js');
let RobotEvent = require('./RobotEvent.js');
let ControllerInputType = require('./ControllerInputType.js');
let Gen3GpioPinId = require('./Gen3GpioPinId.js');
let ControllerConfiguration = require('./ControllerConfiguration.js');
let Mapping = require('./Mapping.js');
let SequenceTaskHandle = require('./SequenceTaskHandle.js');
let MapGroupHandle = require('./MapGroupHandle.js');
let Timeout = require('./Timeout.js');
let TrajectoryErrorType = require('./TrajectoryErrorType.js');
let SwitchControlMapping = require('./SwitchControlMapping.js');
let JointNavigationDirection = require('./JointNavigationDirection.js');
let ProtectionZoneHandle = require('./ProtectionZoneHandle.js');
let Base_SafetyIdentifier = require('./Base_SafetyIdentifier.js');
let AdmittanceMode = require('./AdmittanceMode.js');
let IKData = require('./IKData.js');
let SoundType = require('./SoundType.js');
let Query = require('./Query.js');
let ControllerElementHandle_identifier = require('./ControllerElementHandle_identifier.js');
let NavigationDirection = require('./NavigationDirection.js');
let WifiEncryptionType = require('./WifiEncryptionType.js');
let NetworkNotificationList = require('./NetworkNotificationList.js');
let Xbox360AnalogInputIdentifier = require('./Xbox360AnalogInputIdentifier.js');
let MappingInfoNotificationList = require('./MappingInfoNotificationList.js');
let BackupEvent = require('./BackupEvent.js');
let GpioAction = require('./GpioAction.js');
let SequenceList = require('./SequenceList.js');
let Delay = require('./Delay.js');
let IPv4Configuration = require('./IPv4Configuration.js');
let ControllerList = require('./ControllerList.js');
let SnapshotType = require('./SnapshotType.js');
let Waypoint = require('./Waypoint.js');
let ProtectionZoneNotificationList = require('./ProtectionZoneNotificationList.js');
let BridgeResult = require('./BridgeResult.js');
let SignalQuality = require('./SignalQuality.js');
let Sequence = require('./Sequence.js');
let EmergencyStop = require('./EmergencyStop.js');
let WrenchMode = require('./WrenchMode.js');
let MapGroupList = require('./MapGroupList.js');
let ServoingModeNotificationList = require('./ServoingModeNotificationList.js');
let Pose = require('./Pose.js');
let ControllerHandle = require('./ControllerHandle.js');
let BridgeConfig = require('./BridgeConfig.js');
let NetworkHandle = require('./NetworkHandle.js');
let TrajectoryContinuityMode = require('./TrajectoryContinuityMode.js');
let Snapshot = require('./Snapshot.js');
let ActionNotification = require('./ActionNotification.js');
let ConfigurationChangeNotification = require('./ConfigurationChangeNotification.js');
let ProtectionZoneList = require('./ProtectionZoneList.js');
let GripperRequest = require('./GripperRequest.js');
let ConstrainedPose = require('./ConstrainedPose.js');
let Base_ControlMode = require('./Base_ControlMode.js');
let CartesianLimitation = require('./CartesianLimitation.js');
let WaypointValidationReport = require('./WaypointValidationReport.js');
let Finger = require('./Finger.js');
let Map = require('./Map.js');
let MappingList = require('./MappingList.js');
let SystemTime = require('./SystemTime.js');
let SequenceHandle = require('./SequenceHandle.js');
let WaypointList = require('./WaypointList.js');
let Action = require('./Action.js');
let SequenceTasks = require('./SequenceTasks.js');
let Waypoint_type_of_waypoint = require('./Waypoint_type_of_waypoint.js');
let ConstrainedJointAngles = require('./ConstrainedJointAngles.js');
let TransformationMatrix = require('./TransformationMatrix.js');
let TrajectoryErrorIdentifier = require('./TrajectoryErrorIdentifier.js');
let TrajectoryErrorElement = require('./TrajectoryErrorElement.js');
let ConstrainedJointAngle = require('./ConstrainedJointAngle.js');
let FirmwareBundleVersions = require('./FirmwareBundleVersions.js');
let Xbox360DigitalInputIdentifier = require('./Xbox360DigitalInputIdentifier.js');
let JointLimitation = require('./JointLimitation.js');
let ActionType = require('./ActionType.js');
let ConfigurationChangeNotification_configuration_change = require('./ConfigurationChangeNotification_configuration_change.js');
let ShapeType = require('./ShapeType.js');
let BridgeList = require('./BridgeList.js');
let SequenceTasksPair = require('./SequenceTasksPair.js');
let TrajectoryErrorReport = require('./TrajectoryErrorReport.js');
let IPv4Information = require('./IPv4Information.js');
let NetworkNotification = require('./NetworkNotification.js');
let GpioBehavior = require('./GpioBehavior.js');
let GpioConfigurationList = require('./GpioConfigurationList.js');
let AngularWaypoint = require('./AngularWaypoint.js');
let Twist = require('./Twist.js');
let GpioCommand = require('./GpioCommand.js');
let OperatingModeNotification = require('./OperatingModeNotification.js');
let SequenceInfoNotification = require('./SequenceInfoNotification.js');
let ControllerElementState = require('./ControllerElementState.js');
let AppendActionInformation = require('./AppendActionInformation.js');
let Gripper = require('./Gripper.js');
let CommunicationInterfaceConfiguration = require('./CommunicationInterfaceConfiguration.js');
let Base_ControlModeNotification = require('./Base_ControlModeNotification.js');
let ConstrainedPosition = require('./ConstrainedPosition.js');
let ActivateMapHandle = require('./ActivateMapHandle.js');
let TrajectoryInfo = require('./TrajectoryInfo.js');
let WifiSecurityType = require('./WifiSecurityType.js');
let ActionList = require('./ActionList.js');
let TwistLimitation = require('./TwistLimitation.js');
let SequenceInformation = require('./SequenceInformation.js');
let GripperMode = require('./GripperMode.js');
let Base_Stop = require('./Base_Stop.js');
let SequenceTasksConfiguration = require('./SequenceTasksConfiguration.js');
let WifiInformation = require('./WifiInformation.js');
let RobotEventNotification = require('./RobotEventNotification.js');
let MapList = require('./MapList.js');
let AdvancedSequenceHandle = require('./AdvancedSequenceHandle.js');
let ProtectionZoneEvent = require('./ProtectionZoneEvent.js');
let Base_GpioConfiguration = require('./Base_GpioConfiguration.js');
let GpioPinPropertyFlags = require('./GpioPinPropertyFlags.js');
let RobotEventNotificationList = require('./RobotEventNotificationList.js');
let UserEvent = require('./UserEvent.js');
let Base_Position = require('./Base_Position.js');
let ConfigurationChangeNotificationList = require('./ConfigurationChangeNotificationList.js');
let Admittance = require('./Admittance.js');
let ActionEvent = require('./ActionEvent.js');
let WrenchCommand = require('./WrenchCommand.js');
let ServoingModeInformation = require('./ServoingModeInformation.js');
let ChangeJointSpeeds = require('./ChangeJointSpeeds.js');
let Base_ServiceVersion = require('./Base_ServiceVersion.js');
let TrajectoryInfoType = require('./TrajectoryInfoType.js');
let ChangeTwist = require('./ChangeTwist.js');
let MapEvent = require('./MapEvent.js');
let ControllerState = require('./ControllerState.js');
let JointTrajectoryConstraint = require('./JointTrajectoryConstraint.js');
let UserList = require('./UserList.js');
let FullUserProfile = require('./FullUserProfile.js');
let FullIPv4Configuration = require('./FullIPv4Configuration.js');
let WifiConfiguration = require('./WifiConfiguration.js');
let WifiConfigurationList = require('./WifiConfigurationList.js');
let EventIdSequenceInfoNotification = require('./EventIdSequenceInfoNotification.js');
let OperatingModeInformation = require('./OperatingModeInformation.js');
let RequestedActionType = require('./RequestedActionType.js');
let SafetyNotificationList = require('./SafetyNotificationList.js');
let MappingInfoNotification = require('./MappingInfoNotification.js');
let MappingHandle = require('./MappingHandle.js');
let ControllerEventType = require('./ControllerEventType.js');
let NetworkType = require('./NetworkType.js');
let ArmStateInformation = require('./ArmStateInformation.js');
let ControllerEvent = require('./ControllerEvent.js');
let BaseCyclic_Command = require('./BaseCyclic_Command.js');
let BaseCyclic_ServiceVersion = require('./BaseCyclic_ServiceVersion.js');
let ActuatorCustomData = require('./ActuatorCustomData.js');
let ActuatorCommand = require('./ActuatorCommand.js');
let BaseCyclic_Feedback = require('./BaseCyclic_Feedback.js');
let ActuatorFeedback = require('./ActuatorFeedback.js');
let BaseFeedback = require('./BaseFeedback.js');
let BaseCyclic_CustomData = require('./BaseCyclic_CustomData.js');
let ArmState = require('./ArmState.js');
let NotificationOptions = require('./NotificationOptions.js');
let DeviceHandle = require('./DeviceHandle.js');
let Permission = require('./Permission.js');
let UARTWordLength = require('./UARTWordLength.js');
let UARTConfiguration = require('./UARTConfiguration.js');
let NotificationHandle = require('./NotificationHandle.js');
let SafetyHandle = require('./SafetyHandle.js');
let Connection = require('./Connection.js');
let Empty = require('./Empty.js');
let SafetyStatusValue = require('./SafetyStatusValue.js');
let UARTStopBits = require('./UARTStopBits.js');
let UARTSpeed = require('./UARTSpeed.js');
let DeviceTypes = require('./DeviceTypes.js');
let Unit = require('./Unit.js');
let SafetyNotification = require('./SafetyNotification.js');
let UARTDeviceIdentification = require('./UARTDeviceIdentification.js');
let UserProfileHandle = require('./UserProfileHandle.js');
let UARTParity = require('./UARTParity.js');
let CountryCode = require('./CountryCode.js');
let NotificationType = require('./NotificationType.js');
let CartesianReferenceFrame = require('./CartesianReferenceFrame.js');
let CountryCodeIdentifier = require('./CountryCodeIdentifier.js');
let Timestamp = require('./Timestamp.js');
let ControlConfig_ControlMode = require('./ControlConfig_ControlMode.js');
let ControlConfig_ControlModeNotification = require('./ControlConfig_ControlModeNotification.js');
let GravityVector = require('./GravityVector.js');
let JointAccelerationSoftLimits = require('./JointAccelerationSoftLimits.js');
let AngularTwist = require('./AngularTwist.js');
let ControlConfigurationNotification = require('./ControlConfigurationNotification.js');
let LinearTwist = require('./LinearTwist.js');
let ToolConfiguration = require('./ToolConfiguration.js');
let DesiredSpeeds = require('./DesiredSpeeds.js');
let KinematicLimits = require('./KinematicLimits.js');
let CartesianTransform = require('./CartesianTransform.js');
let ControlConfig_Position = require('./ControlConfig_Position.js');
let ControlConfig_ServiceVersion = require('./ControlConfig_ServiceVersion.js');
let ControlConfig_ControlModeInformation = require('./ControlConfig_ControlModeInformation.js');
let PayloadInformation = require('./PayloadInformation.js');
let TwistAngularSoftLimit = require('./TwistAngularSoftLimit.js');
let JointSpeedSoftLimits = require('./JointSpeedSoftLimits.js');
let ControlConfig_JointSpeeds = require('./ControlConfig_JointSpeeds.js');
let TwistLinearSoftLimit = require('./TwistLinearSoftLimit.js');
let CartesianReferenceFrameInfo = require('./CartesianReferenceFrameInfo.js');
let ControlConfigurationEvent = require('./ControlConfigurationEvent.js');
let KinematicLimitsList = require('./KinematicLimitsList.js');
let SafetyStatus = require('./SafetyStatus.js');
let PartNumber = require('./PartNumber.js');
let ModelNumber = require('./ModelNumber.js');
let SafetyConfiguration = require('./SafetyConfiguration.js');
let MACAddress = require('./MACAddress.js');
let FirmwareVersion = require('./FirmwareVersion.js');
let SafetyInformation = require('./SafetyInformation.js');
let IPv4Settings = require('./IPv4Settings.js');
let SerialNumber = require('./SerialNumber.js');
let CalibrationParameter = require('./CalibrationParameter.js');
let PowerOnSelfTestResult = require('./PowerOnSelfTestResult.js');
let SafetyEnable = require('./SafetyEnable.js');
let DeviceConfig_ServiceVersion = require('./DeviceConfig_ServiceVersion.js');
let RebootRqst = require('./RebootRqst.js');
let BootloaderVersion = require('./BootloaderVersion.js');
let CalibrationResult = require('./CalibrationResult.js');
let Calibration = require('./Calibration.js');
let SafetyThreshold = require('./SafetyThreshold.js');
let CalibrationParameter_value = require('./CalibrationParameter_value.js');
let RunMode = require('./RunMode.js');
let CapSenseRegister = require('./CapSenseRegister.js');
let DeviceConfig_CapSenseConfig = require('./DeviceConfig_CapSenseConfig.js');
let CalibrationElement = require('./CalibrationElement.js');
let CalibrationStatus = require('./CalibrationStatus.js');
let CalibrationItem = require('./CalibrationItem.js');
let PartNumberRevision = require('./PartNumberRevision.js');
let DeviceType = require('./DeviceType.js');
let SafetyInformationList = require('./SafetyInformationList.js');
let RunModes = require('./RunModes.js');
let DeviceConfig_CapSenseMode = require('./DeviceConfig_CapSenseMode.js');
let DeviceConfig_SafetyLimitType = require('./DeviceConfig_SafetyLimitType.js');
let SafetyConfigurationList = require('./SafetyConfigurationList.js');
let DeviceManager_ServiceVersion = require('./DeviceManager_ServiceVersion.js');
let DeviceHandles = require('./DeviceHandles.js');
let GripperConfig_SafetyIdentifier = require('./GripperConfig_SafetyIdentifier.js');
let RobotiqGripperStatusFlags = require('./RobotiqGripperStatusFlags.js');
let GripperCyclic_CustomData = require('./GripperCyclic_CustomData.js');
let CustomDataUnit = require('./CustomDataUnit.js');
let MotorFeedback = require('./MotorFeedback.js');
let GripperCyclic_Command = require('./GripperCyclic_Command.js');
let GripperCyclic_ServiceVersion = require('./GripperCyclic_ServiceVersion.js');
let MotorCommand = require('./MotorCommand.js');
let GripperCyclic_MessageId = require('./GripperCyclic_MessageId.js');
let GripperCyclic_Feedback = require('./GripperCyclic_Feedback.js');
let EthernetDeviceIdentification = require('./EthernetDeviceIdentification.js');
let GPIOPull = require('./GPIOPull.js');
let GPIOIdentifier = require('./GPIOIdentifier.js');
let I2CConfiguration = require('./I2CConfiguration.js');
let EthernetDuplex = require('./EthernetDuplex.js');
let I2CDeviceIdentification = require('./I2CDeviceIdentification.js');
let InterconnectConfig_GPIOConfiguration = require('./InterconnectConfig_GPIOConfiguration.js');
let I2CRegisterAddressSize = require('./I2CRegisterAddressSize.js');
let EthernetSpeed = require('./EthernetSpeed.js');
let GPIOMode = require('./GPIOMode.js');
let UARTPortId = require('./UARTPortId.js');
let GPIOState = require('./GPIOState.js');
let I2CReadParameter = require('./I2CReadParameter.js');
let EthernetDevice = require('./EthernetDevice.js');
let I2CDeviceAddressing = require('./I2CDeviceAddressing.js');
let I2CWriteRegisterParameter = require('./I2CWriteRegisterParameter.js');
let EthernetConfiguration = require('./EthernetConfiguration.js');
let GPIOValue = require('./GPIOValue.js');
let I2CReadRegisterParameter = require('./I2CReadRegisterParameter.js');
let GPIOIdentification = require('./GPIOIdentification.js');
let InterconnectConfig_ServiceVersion = require('./InterconnectConfig_ServiceVersion.js');
let I2CMode = require('./I2CMode.js');
let I2CDevice = require('./I2CDevice.js');
let I2CData = require('./I2CData.js');
let InterconnectConfig_SafetyIdentifier = require('./InterconnectConfig_SafetyIdentifier.js');
let I2CWriteParameter = require('./I2CWriteParameter.js');
let InterconnectCyclic_ServiceVersion = require('./InterconnectCyclic_ServiceVersion.js');
let InterconnectCyclic_CustomData = require('./InterconnectCyclic_CustomData.js');
let InterconnectCyclic_Command = require('./InterconnectCyclic_Command.js');
let InterconnectCyclic_CustomData_tool_customData = require('./InterconnectCyclic_CustomData_tool_customData.js');
let InterconnectCyclic_Command_tool_command = require('./InterconnectCyclic_Command_tool_command.js');
let InterconnectCyclic_Feedback_tool_feedback = require('./InterconnectCyclic_Feedback_tool_feedback.js');
let InterconnectCyclic_Feedback = require('./InterconnectCyclic_Feedback.js');
let InterconnectCyclic_MessageId = require('./InterconnectCyclic_MessageId.js');
let EndEffectorType = require('./EndEffectorType.js');
let ArmLaterality = require('./ArmLaterality.js');
let CompleteProductConfiguration = require('./CompleteProductConfiguration.js');
let BaseType = require('./BaseType.js');
let InterfaceModuleType = require('./InterfaceModuleType.js');
let VisionModuleType = require('./VisionModuleType.js');
let ModelId = require('./ModelId.js');
let ProductConfigurationEndEffectorType = require('./ProductConfigurationEndEffectorType.js');
let WristType = require('./WristType.js');
let BitRate = require('./BitRate.js');
let Sensor = require('./Sensor.js');
let TranslationVector = require('./TranslationVector.js');
let ExtrinsicParameters = require('./ExtrinsicParameters.js');
let FocusPoint = require('./FocusPoint.js');
let DistortionCoefficients = require('./DistortionCoefficients.js');
let VisionConfig_RotationMatrix = require('./VisionConfig_RotationMatrix.js');
let OptionIdentifier = require('./OptionIdentifier.js');
let VisionNotification = require('./VisionNotification.js');
let Option = require('./Option.js');
let ManualFocus = require('./ManualFocus.js');
let Resolution = require('./Resolution.js');
let SensorFocusAction = require('./SensorFocusAction.js');
let OptionInformation = require('./OptionInformation.js');
let VisionConfig_ServiceVersion = require('./VisionConfig_ServiceVersion.js');
let SensorFocusAction_action_parameters = require('./SensorFocusAction_action_parameters.js');
let VisionEvent = require('./VisionEvent.js');
let SensorSettings = require('./SensorSettings.js');
let FrameRate = require('./FrameRate.js');
let FocusAction = require('./FocusAction.js');
let IntrinsicProfileIdentifier = require('./IntrinsicProfileIdentifier.js');
let IntrinsicParameters = require('./IntrinsicParameters.js');
let OptionValue = require('./OptionValue.js');
let VisionConfig_RotationMatrixRow = require('./VisionConfig_RotationMatrixRow.js');
let SensorIdentifier = require('./SensorIdentifier.js');
let FollowCartesianTrajectoryGoal = require('./FollowCartesianTrajectoryGoal.js');
let FollowCartesianTrajectoryActionResult = require('./FollowCartesianTrajectoryActionResult.js');
let FollowCartesianTrajectoryFeedback = require('./FollowCartesianTrajectoryFeedback.js');
let FollowCartesianTrajectoryAction = require('./FollowCartesianTrajectoryAction.js');
let FollowCartesianTrajectoryActionGoal = require('./FollowCartesianTrajectoryActionGoal.js');
let FollowCartesianTrajectoryActionFeedback = require('./FollowCartesianTrajectoryActionFeedback.js');
let FollowCartesianTrajectoryResult = require('./FollowCartesianTrajectoryResult.js');

module.exports = {
  ApiOptions: ApiOptions,
  KortexError: KortexError,
  ErrorCodes: ErrorCodes,
  SubErrorCodes: SubErrorCodes,
  ActuatorConfig_SafetyLimitType: ActuatorConfig_SafetyLimitType,
  CommandMode: CommandMode,
  ActuatorConfig_ServiceVersion: ActuatorConfig_ServiceVersion,
  CustomDataIndex: CustomDataIndex,
  CustomDataSelection: CustomDataSelection,
  AxisOffsets: AxisOffsets,
  ControlLoop: ControlLoop,
  EncoderDerivativeParameters: EncoderDerivativeParameters,
  TorqueCalibration: TorqueCalibration,
  StepResponse: StepResponse,
  VectorDriveParameters: VectorDriveParameters,
  CommandModeInformation: CommandModeInformation,
  Servoing: Servoing,
  ControlLoopSelection: ControlLoopSelection,
  TorqueOffset: TorqueOffset,
  RampResponse: RampResponse,
  PositionCommand: PositionCommand,
  AxisPosition: AxisPosition,
  ActuatorConfig_ControlMode: ActuatorConfig_ControlMode,
  LoopSelection: LoopSelection,
  FrequencyResponse: FrequencyResponse,
  CoggingFeedforwardMode: CoggingFeedforwardMode,
  CoggingFeedforwardModeInformation: CoggingFeedforwardModeInformation,
  ControlLoopParameters: ControlLoopParameters,
  SafetyIdentifierBankA: SafetyIdentifierBankA,
  ActuatorConfig_ControlModeInformation: ActuatorConfig_ControlModeInformation,
  StatusFlags: StatusFlags,
  ActuatorCyclic_MessageId: ActuatorCyclic_MessageId,
  ActuatorCyclic_Command: ActuatorCyclic_Command,
  ActuatorCyclic_Feedback: ActuatorCyclic_Feedback,
  ActuatorCyclic_CustomData: ActuatorCyclic_CustomData,
  CommandFlags: CommandFlags,
  ActuatorCyclic_ServiceVersion: ActuatorCyclic_ServiceVersion,
  SafetyEvent: SafetyEvent,
  FactoryEvent: FactoryEvent,
  ControllerElementHandle: ControllerElementHandle,
  UserNotificationList: UserNotificationList,
  JointTorque: JointTorque,
  ActionNotificationList: ActionNotificationList,
  ControllerNotification_state: ControllerNotification_state,
  ControllerElementEventType: ControllerElementEventType,
  MapHandle: MapHandle,
  ServoingMode: ServoingMode,
  Orientation: Orientation,
  ActuatorInformation: ActuatorInformation,
  JointAngles: JointAngles,
  GpioEvent: GpioEvent,
  JointTorques: JointTorques,
  ActionExecutionState: ActionExecutionState,
  PreComputedJointTrajectoryElement: PreComputedJointTrajectoryElement,
  OperatingMode: OperatingMode,
  Ssid: Ssid,
  ControllerType: ControllerType,
  Base_CapSenseMode: Base_CapSenseMode,
  MapEvent_events: MapEvent_events,
  TwistCommand: TwistCommand,
  BridgeType: BridgeType,
  Wrench: Wrench,
  ControllerConfigurationList: ControllerConfigurationList,
  JointTrajectoryConstraintType: JointTrajectoryConstraintType,
  WifiInformationList: WifiInformationList,
  Base_RotationMatrix: Base_RotationMatrix,
  ArmStateNotification: ArmStateNotification,
  MapElement: MapElement,
  ControllerNotification: ControllerNotification,
  BridgePortConfig: BridgePortConfig,
  SequenceTaskConfiguration: SequenceTaskConfiguration,
  SequenceInfoNotificationList: SequenceInfoNotificationList,
  CartesianTrajectoryConstraint: CartesianTrajectoryConstraint,
  ActionHandle: ActionHandle,
  ServoingModeNotification: ServoingModeNotification,
  WrenchLimitation: WrenchLimitation,
  ZoneShape: ZoneShape,
  NetworkEvent: NetworkEvent,
  JointSpeed: JointSpeed,
  Base_RotationMatrixRow: Base_RotationMatrixRow,
  FirmwareComponentVersion: FirmwareComponentVersion,
  MapGroup: MapGroup,
  ProtectionZoneNotification: ProtectionZoneNotification,
  ConfigurationNotificationEvent: ConfigurationNotificationEvent,
  LedState: LedState,
  SequenceTask: SequenceTask,
  PreComputedJointTrajectory: PreComputedJointTrajectory,
  BridgeIdentifier: BridgeIdentifier,
  WristDigitalInputIdentifier: WristDigitalInputIdentifier,
  ConstrainedOrientation: ConstrainedOrientation,
  OperatingModeNotificationList: OperatingModeNotificationList,
  CartesianWaypoint: CartesianWaypoint,
  BridgeStatus: BridgeStatus,
  SequenceTasksRange: SequenceTasksRange,
  Faults: Faults,
  ProtectionZoneInformation: ProtectionZoneInformation,
  KinematicTrajectoryConstraints: KinematicTrajectoryConstraints,
  Base_ControlModeInformation: Base_ControlModeInformation,
  ControllerNotificationList: ControllerNotificationList,
  LimitationType: LimitationType,
  UserNotification: UserNotification,
  Base_JointSpeeds: Base_JointSpeeds,
  ProtectionZone: ProtectionZone,
  CartesianLimitationList: CartesianLimitationList,
  UserProfileList: UserProfileList,
  CartesianTrajectoryConstraint_type: CartesianTrajectoryConstraint_type,
  UserProfile: UserProfile,
  JointAngle: JointAngle,
  ChangeWrench: ChangeWrench,
  ControllerBehavior: ControllerBehavior,
  Action_action_parameters: Action_action_parameters,
  Point: Point,
  Base_CapSenseConfig: Base_CapSenseConfig,
  FactoryNotification: FactoryNotification,
  PasswordChange: PasswordChange,
  ControlModeNotificationList: ControlModeNotificationList,
  TransformationRow: TransformationRow,
  ControllerConfigurationMode: ControllerConfigurationMode,
  CartesianSpeed: CartesianSpeed,
  GripperCommand: GripperCommand,
  JointsLimitationsList: JointsLimitationsList,
  GpioPinConfiguration: GpioPinConfiguration,
  RobotEvent: RobotEvent,
  ControllerInputType: ControllerInputType,
  Gen3GpioPinId: Gen3GpioPinId,
  ControllerConfiguration: ControllerConfiguration,
  Mapping: Mapping,
  SequenceTaskHandle: SequenceTaskHandle,
  MapGroupHandle: MapGroupHandle,
  Timeout: Timeout,
  TrajectoryErrorType: TrajectoryErrorType,
  SwitchControlMapping: SwitchControlMapping,
  JointNavigationDirection: JointNavigationDirection,
  ProtectionZoneHandle: ProtectionZoneHandle,
  Base_SafetyIdentifier: Base_SafetyIdentifier,
  AdmittanceMode: AdmittanceMode,
  IKData: IKData,
  SoundType: SoundType,
  Query: Query,
  ControllerElementHandle_identifier: ControllerElementHandle_identifier,
  NavigationDirection: NavigationDirection,
  WifiEncryptionType: WifiEncryptionType,
  NetworkNotificationList: NetworkNotificationList,
  Xbox360AnalogInputIdentifier: Xbox360AnalogInputIdentifier,
  MappingInfoNotificationList: MappingInfoNotificationList,
  BackupEvent: BackupEvent,
  GpioAction: GpioAction,
  SequenceList: SequenceList,
  Delay: Delay,
  IPv4Configuration: IPv4Configuration,
  ControllerList: ControllerList,
  SnapshotType: SnapshotType,
  Waypoint: Waypoint,
  ProtectionZoneNotificationList: ProtectionZoneNotificationList,
  BridgeResult: BridgeResult,
  SignalQuality: SignalQuality,
  Sequence: Sequence,
  EmergencyStop: EmergencyStop,
  WrenchMode: WrenchMode,
  MapGroupList: MapGroupList,
  ServoingModeNotificationList: ServoingModeNotificationList,
  Pose: Pose,
  ControllerHandle: ControllerHandle,
  BridgeConfig: BridgeConfig,
  NetworkHandle: NetworkHandle,
  TrajectoryContinuityMode: TrajectoryContinuityMode,
  Snapshot: Snapshot,
  ActionNotification: ActionNotification,
  ConfigurationChangeNotification: ConfigurationChangeNotification,
  ProtectionZoneList: ProtectionZoneList,
  GripperRequest: GripperRequest,
  ConstrainedPose: ConstrainedPose,
  Base_ControlMode: Base_ControlMode,
  CartesianLimitation: CartesianLimitation,
  WaypointValidationReport: WaypointValidationReport,
  Finger: Finger,
  Map: Map,
  MappingList: MappingList,
  SystemTime: SystemTime,
  SequenceHandle: SequenceHandle,
  WaypointList: WaypointList,
  Action: Action,
  SequenceTasks: SequenceTasks,
  Waypoint_type_of_waypoint: Waypoint_type_of_waypoint,
  ConstrainedJointAngles: ConstrainedJointAngles,
  TransformationMatrix: TransformationMatrix,
  TrajectoryErrorIdentifier: TrajectoryErrorIdentifier,
  TrajectoryErrorElement: TrajectoryErrorElement,
  ConstrainedJointAngle: ConstrainedJointAngle,
  FirmwareBundleVersions: FirmwareBundleVersions,
  Xbox360DigitalInputIdentifier: Xbox360DigitalInputIdentifier,
  JointLimitation: JointLimitation,
  ActionType: ActionType,
  ConfigurationChangeNotification_configuration_change: ConfigurationChangeNotification_configuration_change,
  ShapeType: ShapeType,
  BridgeList: BridgeList,
  SequenceTasksPair: SequenceTasksPair,
  TrajectoryErrorReport: TrajectoryErrorReport,
  IPv4Information: IPv4Information,
  NetworkNotification: NetworkNotification,
  GpioBehavior: GpioBehavior,
  GpioConfigurationList: GpioConfigurationList,
  AngularWaypoint: AngularWaypoint,
  Twist: Twist,
  GpioCommand: GpioCommand,
  OperatingModeNotification: OperatingModeNotification,
  SequenceInfoNotification: SequenceInfoNotification,
  ControllerElementState: ControllerElementState,
  AppendActionInformation: AppendActionInformation,
  Gripper: Gripper,
  CommunicationInterfaceConfiguration: CommunicationInterfaceConfiguration,
  Base_ControlModeNotification: Base_ControlModeNotification,
  ConstrainedPosition: ConstrainedPosition,
  ActivateMapHandle: ActivateMapHandle,
  TrajectoryInfo: TrajectoryInfo,
  WifiSecurityType: WifiSecurityType,
  ActionList: ActionList,
  TwistLimitation: TwistLimitation,
  SequenceInformation: SequenceInformation,
  GripperMode: GripperMode,
  Base_Stop: Base_Stop,
  SequenceTasksConfiguration: SequenceTasksConfiguration,
  WifiInformation: WifiInformation,
  RobotEventNotification: RobotEventNotification,
  MapList: MapList,
  AdvancedSequenceHandle: AdvancedSequenceHandle,
  ProtectionZoneEvent: ProtectionZoneEvent,
  Base_GpioConfiguration: Base_GpioConfiguration,
  GpioPinPropertyFlags: GpioPinPropertyFlags,
  RobotEventNotificationList: RobotEventNotificationList,
  UserEvent: UserEvent,
  Base_Position: Base_Position,
  ConfigurationChangeNotificationList: ConfigurationChangeNotificationList,
  Admittance: Admittance,
  ActionEvent: ActionEvent,
  WrenchCommand: WrenchCommand,
  ServoingModeInformation: ServoingModeInformation,
  ChangeJointSpeeds: ChangeJointSpeeds,
  Base_ServiceVersion: Base_ServiceVersion,
  TrajectoryInfoType: TrajectoryInfoType,
  ChangeTwist: ChangeTwist,
  MapEvent: MapEvent,
  ControllerState: ControllerState,
  JointTrajectoryConstraint: JointTrajectoryConstraint,
  UserList: UserList,
  FullUserProfile: FullUserProfile,
  FullIPv4Configuration: FullIPv4Configuration,
  WifiConfiguration: WifiConfiguration,
  WifiConfigurationList: WifiConfigurationList,
  EventIdSequenceInfoNotification: EventIdSequenceInfoNotification,
  OperatingModeInformation: OperatingModeInformation,
  RequestedActionType: RequestedActionType,
  SafetyNotificationList: SafetyNotificationList,
  MappingInfoNotification: MappingInfoNotification,
  MappingHandle: MappingHandle,
  ControllerEventType: ControllerEventType,
  NetworkType: NetworkType,
  ArmStateInformation: ArmStateInformation,
  ControllerEvent: ControllerEvent,
  BaseCyclic_Command: BaseCyclic_Command,
  BaseCyclic_ServiceVersion: BaseCyclic_ServiceVersion,
  ActuatorCustomData: ActuatorCustomData,
  ActuatorCommand: ActuatorCommand,
  BaseCyclic_Feedback: BaseCyclic_Feedback,
  ActuatorFeedback: ActuatorFeedback,
  BaseFeedback: BaseFeedback,
  BaseCyclic_CustomData: BaseCyclic_CustomData,
  ArmState: ArmState,
  NotificationOptions: NotificationOptions,
  DeviceHandle: DeviceHandle,
  Permission: Permission,
  UARTWordLength: UARTWordLength,
  UARTConfiguration: UARTConfiguration,
  NotificationHandle: NotificationHandle,
  SafetyHandle: SafetyHandle,
  Connection: Connection,
  Empty: Empty,
  SafetyStatusValue: SafetyStatusValue,
  UARTStopBits: UARTStopBits,
  UARTSpeed: UARTSpeed,
  DeviceTypes: DeviceTypes,
  Unit: Unit,
  SafetyNotification: SafetyNotification,
  UARTDeviceIdentification: UARTDeviceIdentification,
  UserProfileHandle: UserProfileHandle,
  UARTParity: UARTParity,
  CountryCode: CountryCode,
  NotificationType: NotificationType,
  CartesianReferenceFrame: CartesianReferenceFrame,
  CountryCodeIdentifier: CountryCodeIdentifier,
  Timestamp: Timestamp,
  ControlConfig_ControlMode: ControlConfig_ControlMode,
  ControlConfig_ControlModeNotification: ControlConfig_ControlModeNotification,
  GravityVector: GravityVector,
  JointAccelerationSoftLimits: JointAccelerationSoftLimits,
  AngularTwist: AngularTwist,
  ControlConfigurationNotification: ControlConfigurationNotification,
  LinearTwist: LinearTwist,
  ToolConfiguration: ToolConfiguration,
  DesiredSpeeds: DesiredSpeeds,
  KinematicLimits: KinematicLimits,
  CartesianTransform: CartesianTransform,
  ControlConfig_Position: ControlConfig_Position,
  ControlConfig_ServiceVersion: ControlConfig_ServiceVersion,
  ControlConfig_ControlModeInformation: ControlConfig_ControlModeInformation,
  PayloadInformation: PayloadInformation,
  TwistAngularSoftLimit: TwistAngularSoftLimit,
  JointSpeedSoftLimits: JointSpeedSoftLimits,
  ControlConfig_JointSpeeds: ControlConfig_JointSpeeds,
  TwistLinearSoftLimit: TwistLinearSoftLimit,
  CartesianReferenceFrameInfo: CartesianReferenceFrameInfo,
  ControlConfigurationEvent: ControlConfigurationEvent,
  KinematicLimitsList: KinematicLimitsList,
  SafetyStatus: SafetyStatus,
  PartNumber: PartNumber,
  ModelNumber: ModelNumber,
  SafetyConfiguration: SafetyConfiguration,
  MACAddress: MACAddress,
  FirmwareVersion: FirmwareVersion,
  SafetyInformation: SafetyInformation,
  IPv4Settings: IPv4Settings,
  SerialNumber: SerialNumber,
  CalibrationParameter: CalibrationParameter,
  PowerOnSelfTestResult: PowerOnSelfTestResult,
  SafetyEnable: SafetyEnable,
  DeviceConfig_ServiceVersion: DeviceConfig_ServiceVersion,
  RebootRqst: RebootRqst,
  BootloaderVersion: BootloaderVersion,
  CalibrationResult: CalibrationResult,
  Calibration: Calibration,
  SafetyThreshold: SafetyThreshold,
  CalibrationParameter_value: CalibrationParameter_value,
  RunMode: RunMode,
  CapSenseRegister: CapSenseRegister,
  DeviceConfig_CapSenseConfig: DeviceConfig_CapSenseConfig,
  CalibrationElement: CalibrationElement,
  CalibrationStatus: CalibrationStatus,
  CalibrationItem: CalibrationItem,
  PartNumberRevision: PartNumberRevision,
  DeviceType: DeviceType,
  SafetyInformationList: SafetyInformationList,
  RunModes: RunModes,
  DeviceConfig_CapSenseMode: DeviceConfig_CapSenseMode,
  DeviceConfig_SafetyLimitType: DeviceConfig_SafetyLimitType,
  SafetyConfigurationList: SafetyConfigurationList,
  DeviceManager_ServiceVersion: DeviceManager_ServiceVersion,
  DeviceHandles: DeviceHandles,
  GripperConfig_SafetyIdentifier: GripperConfig_SafetyIdentifier,
  RobotiqGripperStatusFlags: RobotiqGripperStatusFlags,
  GripperCyclic_CustomData: GripperCyclic_CustomData,
  CustomDataUnit: CustomDataUnit,
  MotorFeedback: MotorFeedback,
  GripperCyclic_Command: GripperCyclic_Command,
  GripperCyclic_ServiceVersion: GripperCyclic_ServiceVersion,
  MotorCommand: MotorCommand,
  GripperCyclic_MessageId: GripperCyclic_MessageId,
  GripperCyclic_Feedback: GripperCyclic_Feedback,
  EthernetDeviceIdentification: EthernetDeviceIdentification,
  GPIOPull: GPIOPull,
  GPIOIdentifier: GPIOIdentifier,
  I2CConfiguration: I2CConfiguration,
  EthernetDuplex: EthernetDuplex,
  I2CDeviceIdentification: I2CDeviceIdentification,
  InterconnectConfig_GPIOConfiguration: InterconnectConfig_GPIOConfiguration,
  I2CRegisterAddressSize: I2CRegisterAddressSize,
  EthernetSpeed: EthernetSpeed,
  GPIOMode: GPIOMode,
  UARTPortId: UARTPortId,
  GPIOState: GPIOState,
  I2CReadParameter: I2CReadParameter,
  EthernetDevice: EthernetDevice,
  I2CDeviceAddressing: I2CDeviceAddressing,
  I2CWriteRegisterParameter: I2CWriteRegisterParameter,
  EthernetConfiguration: EthernetConfiguration,
  GPIOValue: GPIOValue,
  I2CReadRegisterParameter: I2CReadRegisterParameter,
  GPIOIdentification: GPIOIdentification,
  InterconnectConfig_ServiceVersion: InterconnectConfig_ServiceVersion,
  I2CMode: I2CMode,
  I2CDevice: I2CDevice,
  I2CData: I2CData,
  InterconnectConfig_SafetyIdentifier: InterconnectConfig_SafetyIdentifier,
  I2CWriteParameter: I2CWriteParameter,
  InterconnectCyclic_ServiceVersion: InterconnectCyclic_ServiceVersion,
  InterconnectCyclic_CustomData: InterconnectCyclic_CustomData,
  InterconnectCyclic_Command: InterconnectCyclic_Command,
  InterconnectCyclic_CustomData_tool_customData: InterconnectCyclic_CustomData_tool_customData,
  InterconnectCyclic_Command_tool_command: InterconnectCyclic_Command_tool_command,
  InterconnectCyclic_Feedback_tool_feedback: InterconnectCyclic_Feedback_tool_feedback,
  InterconnectCyclic_Feedback: InterconnectCyclic_Feedback,
  InterconnectCyclic_MessageId: InterconnectCyclic_MessageId,
  EndEffectorType: EndEffectorType,
  ArmLaterality: ArmLaterality,
  CompleteProductConfiguration: CompleteProductConfiguration,
  BaseType: BaseType,
  InterfaceModuleType: InterfaceModuleType,
  VisionModuleType: VisionModuleType,
  ModelId: ModelId,
  ProductConfigurationEndEffectorType: ProductConfigurationEndEffectorType,
  WristType: WristType,
  BitRate: BitRate,
  Sensor: Sensor,
  TranslationVector: TranslationVector,
  ExtrinsicParameters: ExtrinsicParameters,
  FocusPoint: FocusPoint,
  DistortionCoefficients: DistortionCoefficients,
  VisionConfig_RotationMatrix: VisionConfig_RotationMatrix,
  OptionIdentifier: OptionIdentifier,
  VisionNotification: VisionNotification,
  Option: Option,
  ManualFocus: ManualFocus,
  Resolution: Resolution,
  SensorFocusAction: SensorFocusAction,
  OptionInformation: OptionInformation,
  VisionConfig_ServiceVersion: VisionConfig_ServiceVersion,
  SensorFocusAction_action_parameters: SensorFocusAction_action_parameters,
  VisionEvent: VisionEvent,
  SensorSettings: SensorSettings,
  FrameRate: FrameRate,
  FocusAction: FocusAction,
  IntrinsicProfileIdentifier: IntrinsicProfileIdentifier,
  IntrinsicParameters: IntrinsicParameters,
  OptionValue: OptionValue,
  VisionConfig_RotationMatrixRow: VisionConfig_RotationMatrixRow,
  SensorIdentifier: SensorIdentifier,
  FollowCartesianTrajectoryGoal: FollowCartesianTrajectoryGoal,
  FollowCartesianTrajectoryActionResult: FollowCartesianTrajectoryActionResult,
  FollowCartesianTrajectoryFeedback: FollowCartesianTrajectoryFeedback,
  FollowCartesianTrajectoryAction: FollowCartesianTrajectoryAction,
  FollowCartesianTrajectoryActionGoal: FollowCartesianTrajectoryActionGoal,
  FollowCartesianTrajectoryActionFeedback: FollowCartesianTrajectoryActionFeedback,
  FollowCartesianTrajectoryResult: FollowCartesianTrajectoryResult,
};
