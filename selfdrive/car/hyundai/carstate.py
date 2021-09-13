import copy
from cereal import car
from selfdrive.car.hyundai.values import DBC, STEER_THRESHOLD, FEATURES, EV_CAR, HYBRID_CAR, Buttons
from selfdrive.car.interfaces import CarStateBase
from opendbc.can.parser import CANParser
from opendbc.can.can_define import CANDefine
from selfdrive.config import Conversions as CV

import common.loger as trace1

GearShifter = car.CarState.GearShifter



class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    can_define = CANDefine(DBC[CP.carFingerprint]["pt"])

    if self.CP.carFingerprint in FEATURES["use_cluster_gears"]:
      self.shifter_values = can_define.dv["CLU15"]["CF_Clu_Gear"]
    elif self.CP.carFingerprint in FEATURES["use_tcu_gears"]:
      self.shifter_values = can_define.dv["TCU12"]["CUR_GR"]
    else:  # preferred and elect gear methods use same definition
      self.shifter_values = can_define.dv["LVR12"]["CF_Lvr_Gear"]

    # atom
    self.cruise_buttons = 0
    self.cruise_buttons_time = 0
    self.time_delay_int = 0
    self.VSetDis = 0
    self.clu_Vanz = 0

    # acc button 
    self.prev_clu_CruiseSwState = 0
    self.prev_acc_active = 0
    self.prev_acc_set_btn = False
    self.acc_active = 0
    self.cruise_set_speed_kph = 0
    self.cruise_set_mode = 0      # 모드 설정.
    self.gasPressed = False

    # engage button
    self.cruise_available = False
    self.acc_mode = False
    self.engage_enable = False
    self.enagage_status = 0
    self.cruise_buttons_old = 0

    self.time_break = 0

  def engage_disable( self, ret ):
    steeringAngleDeg = abs( ret.steeringAngleDeg )

    if ret.brakePressed:
      self.time_break = 500
    elif self.time_break > 0:
      self.time_break -= 1

    limitAngleDeg = 50
    if self.time_break:
      limitAngleDeg = 10

    if not self.acc_mode and self.clu_Vanz < 40 and steeringAngleDeg > limitAngleDeg and ret.steeringPressed:
       return True

    return False

  def engage_control( self, ret, c ):
    left_lane = c.hudControl.leftLaneVisible 
    right_lane = c.hudControl.rightLaneVisible     
    status_flag = 0
    if not ret.cruiseState.available or ret.gearShifter != GearShifter.drive or ret.seatbeltUnlatched or ret.doorOpen:
      status_flag = 1
      self.enagage_status = 0
      self.engage_enable = False
      self.time_delay_int = 100
    elif self.acc_mode:
      self.enagage_status = 2
      self.engage_enable = True

    engage_disable_status = self.engage_disable( ret )
    if self.cruise_buttons_old == self.cruise_buttons:
      if self.engage_enable:
        if engage_disable_status:
            self.engage_enable = False
        return self.engage_enable
      elif self.time_delay_int > 0:
        self.time_delay_int -= 1

      if self.time_delay_int > 100:
        pass
      elif ret.vEgo < 5 or not left_lane or not right_lane or ret.steeringPressed or ret.leftBlinker or ret.rightBlinker:  # 15 km/h
        self.time_delay_int = 100
      elif self.time_delay_int <= 0 and  not self.gasPressed:
        self.engage_enable = True

      return  self.engage_enable

    self.cruise_buttons_old = self.cruise_buttons

    if status_flag == 1:
      self.engage_enable = False
    elif self.acc_mode:
      return  self.engage_enable
    elif self.cruise_buttons == Buttons.GAP_DIST:
      self.engage_enable = True
      self.enagage_status = 1
    elif self.cruise_buttons == Buttons.CANCEL:
      self.enagage_status = 0
      self.time_delay_int = 1000
      self.engage_enable = False

    return  self.engage_enable

  #@staticmethod
  def cruise_speed_button( self ):
    if self.prev_acc_active != self.acc_active:
      self.prev_acc_active = self.acc_active
      self.cruise_set_speed_kph = self.clu_Vanz

    set_speed_kph = self.cruise_set_speed_kph
    if not self.cruise_available:
      if self.prev_clu_CruiseSwState != self.cruise_buttons:
        self.prev_clu_CruiseSwState = self.cruise_buttons
        if self.cruise_buttons == Buttons.GAP_DIST:
          self.cruise_set_mode += 1
          if self.cruise_set_mode > 2:
            self.cruise_set_mode = 0
      return self.cruise_set_speed_kph


    if not self.prev_acc_set_btn:
      self.prev_acc_set_btn = self.acc_active
      if self.cruise_buttons == Buttons.RES_ACCEL:   # up 
        self.cruise_set_speed_kph = self.VSetDis
      else:
        self.cruise_set_speed_kph = self.clu_Vanz
      return self.cruise_set_speed_kph

    elif self.prev_acc_set_btn != self.acc_active:
      self.prev_acc_set_btn = self.acc_active

    if self.cruise_buttons:
      self.cruise_buttons_time += 1
    else:
      self.cruise_buttons_time = 0
  
     
    if self.cruise_buttons_time >= 60:
      self.cruise_set_speed_kph = self.VSetDis

    if self.prev_clu_CruiseSwState == self.cruise_buttons:
      return set_speed_kph
    self.prev_clu_CruiseSwState = self.cruise_buttons

    if self.cruise_buttons == Buttons.RES_ACCEL:   # up 
      set_speed_kph +=  1
    elif self.cruise_buttons == Buttons.SET_DECEL:  # dn
      if self.gasPressed:
        set_speed_kph = self.clu_Vanz + 1
      else:
        set_speed_kph -=  1

    if set_speed_kph < 30:
      set_speed_kph = 30

    self.cruise_set_speed_kph = set_speed_kph
    return  set_speed_kph



  # TPMS code added from OPKR
  def update_tpms(self, cp, ret ):
    unit_ratio = 1.0
    unit = cp.vl["TPMS11"]["UNIT"]
    ret.tpms.fl = cp.vl["TPMS11"]['PRESSURE_FL']
    ret.tpms.fr = cp.vl["TPMS11"]['PRESSURE_FR']
    ret.tpms.rl = cp.vl["TPMS11"]['PRESSURE_RL']
    ret.tpms.rr = cp.vl["TPMS11"]['PRESSURE_RR']

    if unit == 1.0:
      unit_ratio = 0.72519
    elif unit == 2.0:
      unit_ratio = 1.45038

    ret.tpms.fl *= unit_ratio
    ret.tpms.fr *= unit_ratio
    ret.tpms.rl *= unit_ratio
    ret.tpms.rr *= unit_ratio
    return ret

  def update(self, cp, cp_cam, c):
    ret = car.CarState.new_message()

    ret.doorOpen = any([cp.vl["CGW1"]["CF_Gway_DrvDrSw"], cp.vl["CGW1"]["CF_Gway_AstDrSw"],
                        cp.vl["CGW2"]["CF_Gway_RLDrSw"], cp.vl["CGW2"]["CF_Gway_RRDrSw"]])

    ret.seatbeltUnlatched = cp.vl["CGW1"]["CF_Gway_DrvSeatBeltSw"] == 0

    ret.wheelSpeeds.fl = cp.vl["WHL_SPD11"]["WHL_SPD_FL"] * CV.KPH_TO_MS
    ret.wheelSpeeds.fr = cp.vl["WHL_SPD11"]["WHL_SPD_FR"] * CV.KPH_TO_MS
    ret.wheelSpeeds.rl = cp.vl["WHL_SPD11"]["WHL_SPD_RL"] * CV.KPH_TO_MS
    ret.wheelSpeeds.rr = cp.vl["WHL_SPD11"]["WHL_SPD_RR"] * CV.KPH_TO_MS
    ret.vEgoRaw = (ret.wheelSpeeds.fl + ret.wheelSpeeds.fr + ret.wheelSpeeds.rl + ret.wheelSpeeds.rr) / 4.
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)

    ret.standstill = ret.vEgoRaw < 0.1

    ret.steeringAngleDeg = cp.vl["SAS11"]["SAS_Angle"]
    ret.steeringRateDeg = cp.vl["SAS11"]["SAS_Speed"]
    ret.yawRate = cp.vl["ESP12"]["YAW_RATE"]
    ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_lamp(
      50, cp.vl["CGW1"]["CF_Gway_TurnSigLh"], cp.vl["CGW1"]["CF_Gway_TurnSigRh"])
    ret.steeringTorque = cp.vl["MDPS12"]["CR_Mdps_StrColTq"]
    ret.steeringTorqueEps = cp.vl["MDPS12"]["CR_Mdps_OutTq"]
    ret.steeringPressed = abs(ret.steeringTorque) > STEER_THRESHOLD
    ret.steerWarning = cp.vl["MDPS12"]["CF_Mdps_ToiUnavail"] != 0 or cp.vl["MDPS12"]["CF_Mdps_ToiFlt"] != 0

    # cruise state
    self.VSetDis = cp.vl["SCC11"]["VSetDis"]   # kph   크루즈 설정 속도.
    self.clu_Vanz = cp.vl["CLU11"]["CF_Clu_Vanz"]  #kph  현재 차량의 속도.
    self.acc_active = (cp.vl["SCC12"]['ACCMode'] != 0)
    ret.vEgo = self.clu_Vanz * CV.KPH_TO_MS
    ret.cruiseState.accActive = self.acc_active
    ret.cruiseState.gapSet = cp.vl["SCC11"]['TauGapSet']
    ret.cruiseState.cruiseSwState = self.cruise_buttons
    ret.cruiseState.modeSel = self.cruise_set_mode


    self.cruise_available = cp.vl["SCC11"]["MainMode_ACC"] == 1
    self.acc_mode = cp.vl["SCC12"]["ACCMode"] != 0
    if self.cruise_set_mode == 2:
      ret.cruiseState.available = False
    else:
      ret.cruiseState.available = self.cruise_available
    ret.cruiseState.standstill = cp.vl["SCC11"]["SCCInfoDisplay"] == 4.



    


    set_speed = self.cruise_speed_button()
    if self.acc_active:
      speed_conv = CV.MPH_TO_MS if cp.vl["CLU11"]["CF_Clu_SPEED_UNIT"] else CV.KPH_TO_MS
      ret.cruiseState.speed = set_speed * speed_conv
    else:
      ret.cruiseState.speed = 0

    # TODO: Find brake pressure
    ret.brake = 0
    ret.brakePressed = cp.vl["TCS13"]["DriverBraking"] != 0
    ret.brakeLightsDEPRECATED = bool(cp.vl["TCS13"]['BrakeLight'])

    if self.CP.carFingerprint in (HYBRID_CAR | EV_CAR):
      if self.CP.carFingerprint in HYBRID_CAR:
        ret.gas = cp.vl["E_EMS11"]["CR_Vcu_AccPedDep_Pos"] / 254.
      else:
        ret.gas = cp.vl["E_EMS11"]["Accel_Pedal_Pos"] / 254.
      ret.gasPressed = ret.gas > 0
    else:
      ret.gas = cp.vl["EMS12"]["PV_AV_CAN"] / 100.
      ret.gasPressed = bool(cp.vl["EMS16"]["CF_Ems_AclAct"])

    self.gasPressed = ret.gasPressed
    # Gear Selection via Cluster - For those Kia/Hyundai which are not fully discovered, we can use the Cluster Indicator for Gear Selection,
    # as this seems to be standard over all cars, but is not the preferred method.
    if self.CP.carFingerprint in FEATURES["use_cluster_gears"]:
      gear = cp.vl["CLU15"]["CF_Clu_Gear"]
    elif self.CP.carFingerprint in FEATURES["use_tcu_gears"]:
      gear = cp.vl["TCU12"]["CUR_GR"]
    elif self.CP.carFingerprint in FEATURES["use_elect_gears"]:
      gear = cp.vl["ELECT_GEAR"]["Elect_Gear_Shifter"]
    else:
      gear = cp.vl["LVR12"]["CF_Lvr_Gear"]

    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(gear))

    if self.CP.carFingerprint in FEATURES["use_fca"]:
      ret.stockAeb = cp.vl["FCA11"]["FCA_CmdAct"] != 0
      ret.stockFcw = cp.vl["FCA11"]["CF_VSM_Warn"] == 2
    else:
      ret.stockAeb = cp.vl["SCC12"]["AEB_CmdAct"] != 0
      ret.stockFcw = cp.vl["SCC12"]["CF_VSM_Warn"] == 2

    if self.CP.enableBsm:
      ret.leftBlindspot = cp.vl["LCA11"]["CF_Lca_IndLeft"] != 0
      ret.rightBlindspot = cp.vl["LCA11"]["CF_Lca_IndRight"] != 0

    ret = self.update_tpms( cp, ret )
    ret.cruiseState.enabled = self.engage_control( ret, c )

    # save the entire LKAS11 and CLU11
    self.lfahda = copy.copy(cp_cam.vl["LFAHDA_MFC"])
    self.lkas11 = copy.copy(cp_cam.vl["LKAS11"])
    self.clu11 = copy.copy(cp.vl["CLU11"])
    self.mdps12 = copy.copy(cp.vl["MDPS12"])
    self.park_brake = cp.vl["TCS13"]["PBRAKE_ACT"] == 1
    self.steer_state = cp.vl["MDPS12"]["CF_Mdps_ToiActive"]  # 0 NOT ACTIVE, 1 ACTIVE
    self.lead_distance = cp.vl["SCC11"]["ACC_ObjDist"]
    self.brake_hold = cp.vl["TCS15"]["AVH_LAMP"] == 2 # 0 OFF, 1 ERROR, 2 ACTIVE, 3 READY
    self.brake_error = cp.vl["TCS13"]["ACCEnable"] != 0 # 0 ACC CONTROL ENABLED, 1-3 ACC CONTROL DISABLED
    self.prev_cruise_buttons = self.cruise_buttons
    self.cruise_buttons = cp.vl["CLU11"]["CF_Clu_CruiseSwState"]

    self.lkas_button_on = cp_cam.vl["LKAS11"]["CF_Lkas_LdwsSysState"]
    self.is_highway = self.lfahda["HDA_Icon_State"] != 0.

    self.Elect_Gear_Step = cp.vl["ELECT_GEAR"]["Elect_Gear_Step"]
    self.SpeedLim_Nav_Clu = cp.vl["Navi_HU"]["SpeedLim_Nav_Clu"]

    return ret


  @staticmethod
  def get_can_parser(CP):
    signals = [
      # sig_name, sig_address, default
      ("WHL_SPD_FL", "WHL_SPD11", 0),
      ("WHL_SPD_FR", "WHL_SPD11", 0),
      ("WHL_SPD_RL", "WHL_SPD11", 0),
      ("WHL_SPD_RR", "WHL_SPD11", 0),

      ("YAW_RATE", "ESP12", 0),

      ("CF_Gway_DrvSeatBeltInd", "CGW4", 1),

      ("CF_Gway_DrvSeatBeltSw", "CGW1", 0),
      ("CF_Gway_DrvDrSw", "CGW1", 0),       # Driver Door
      ("CF_Gway_AstDrSw", "CGW1", 0),       # Passenger door
      ("CF_Gway_RLDrSw", "CGW2", 0),        # Rear reft door
      ("CF_Gway_RRDrSw", "CGW2", 0),        # Rear right door
      ("CF_Gway_TurnSigLh", "CGW1", 0),
      ("CF_Gway_TurnSigRh", "CGW1", 0),
      ("CF_Gway_ParkBrakeSw", "CGW1", 0),

      ("CYL_PRES", "ESP12", 0),

      ("CF_Clu_CruiseSwState", "CLU11", 0),
      ("CF_Clu_CruiseSwMain", "CLU11", 0),
      ("CF_Clu_SldMainSW", "CLU11", 0),
      ("CF_Clu_ParityBit1", "CLU11", 0),
      ("CF_Clu_VanzDecimal" , "CLU11", 0),
      ("CF_Clu_Vanz", "CLU11", 0),
      ("CF_Clu_SPEED_UNIT", "CLU11", 0),
      ("CF_Clu_DetentOut", "CLU11", 0),
      ("CF_Clu_RheostatLevel", "CLU11", 0),
      ("CF_Clu_CluInfo", "CLU11", 0),
      ("CF_Clu_AmpInfo", "CLU11", 0),
      ("CF_Clu_AliveCnt1", "CLU11", 0),

      ("ACCEnable", "TCS13", 0),
      ("ACC_REQ", "TCS13", 0),
      ("DriverBraking", "TCS13", 0),
      ("BrakeLight", "TCS13", 0),      
      ("StandStill", "TCS13", 0),
      ("PBRAKE_ACT", "TCS13", 0),

      ("ESC_Off_Step", "TCS15", 0),
      ("AVH_LAMP", "TCS15", 0),

      ("CR_Mdps_StrColTq", "MDPS12", 0),
      ("CF_Mdps_ToiActive", "MDPS12", 0),
      ("CF_Mdps_ToiUnavail", "MDPS12", 0),
      ("CF_Mdps_ToiFlt", "MDPS12", 0),
      ("CR_Mdps_OutTq", "MDPS12", 0),

      ("CF_Mdps_MsgCount2", "MDPS12", 0),  #
      ("CF_Mdps_Chksum2", "MDPS12", 0),    #



      ("SAS_Angle", "SAS11", 0),
      ("SAS_Speed", "SAS11", 0),

      ("MainMode_ACC", "SCC11", 0),
      ("VSetDis", "SCC11", 0),
      ("SCCInfoDisplay", "SCC11", 0),
      ("ACC_ObjDist", "SCC11", 0),
      ("ACCMode", "SCC12", 1),

      ("Navi_SCC_Camera_Act", "SCC11", 0),
      ("TauGapSet", "SCC11", 4),




      # TPMS
      ("UNIT", "TPMS11", 0),
      ("PRESSURE_FL", "TPMS11", 0),
      ("PRESSURE_FR", "TPMS11", 0),
      ("PRESSURE_RL", "TPMS11", 0),
      ("PRESSURE_RR", "TPMS11", 0),

      ("SpeedLim_Nav_Clu", "Navi_HU", 0),
    ]

    checks = [
      # address, frequency
      ("MDPS12", 50),
      ("TCS13", 50),
      ("TCS15", 10),
      ("CLU11", 50),
      ("ESP12", 100),
      ("CGW1", 10),
      ("CGW2", 5),
      ("CGW4", 5),
      ("WHL_SPD11", 50),
      ("SAS11", 100),

      ("SCC11", 50),
      ("SCC12", 50),

      ("TPMS11", 5),
      ("Navi_HU", 5),
    ]


    if CP.enableBsm:
      signals += [
        ("CF_Lca_IndLeft", "LCA11", 0),
        ("CF_Lca_IndRight", "LCA11", 0),
      ]
      checks += [("LCA11", 50)]

    if CP.carFingerprint in (HYBRID_CAR | EV_CAR):
      if CP.carFingerprint in HYBRID_CAR:
        signals += [
          ("CR_Vcu_AccPedDep_Pos", "E_EMS11", 0)
        ]
      else:
        signals += [
          ("Accel_Pedal_Pos", "E_EMS11", 0)
        ]
      checks += [
        ("E_EMS11", 50),
      ]
    else:
      signals += [
        ("PV_AV_CAN", "EMS12", 0),
        ("CF_Ems_AclAct", "EMS16", 0),
      ]
      checks += [
        ("EMS12", 100),
        ("EMS16", 100),
      ]

    if CP.carFingerprint in FEATURES["use_cluster_gears"]:
      signals += [
        ("CF_Clu_Gear", "CLU15", 0),
      ]
      checks += [
        ("CLU15", 5)
      ]
    elif CP.carFingerprint in FEATURES["use_tcu_gears"]:
      signals += [
        ("CUR_GR", "TCU12", 0)
      ]
      checks += [
        ("TCU12", 100)
      ]
    elif CP.carFingerprint in FEATURES["use_elect_gears"]:
      signals += [("Elect_Gear_Shifter", "ELECT_GEAR", 0)]
      signals += [("Elect_Gear_Step", "ELECT_GEAR", 0)]
      checks += [("ELECT_GEAR", 20)]
    else:
      signals += [
        ("CF_Lvr_Gear", "LVR12", 0)
      ]
      checks += [
        ("LVR12", 100)
      ]

    if CP.carFingerprint in FEATURES["use_fca"]:
      signals += [
        ("FCA_CmdAct", "FCA11", 0),
        ("CF_VSM_Warn", "FCA11", 0),
      ]
      if not CP.openpilotLongitudinalControl:
        checks += [("FCA11", 50)]
    else:
      signals += [
        ("AEB_CmdAct", "SCC12", 0),
        ("CF_VSM_Warn", "SCC12", 0),
      ]

    return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, 0)

  @staticmethod
  def get_cam_can_parser(CP):

    signals = [
      # sig_name, sig_address, default
      ("CF_Lkas_LdwsActivemode", "LKAS11", 0),
      ("CF_Lkas_LdwsSysState", "LKAS11", 0),
      ("CF_Lkas_SysWarning", "LKAS11", 0),
      ("CF_Lkas_LdwsLHWarning", "LKAS11", 0),
      ("CF_Lkas_LdwsRHWarning", "LKAS11", 0),
      ("CF_Lkas_HbaLamp", "LKAS11", 0),
      ("CF_Lkas_FcwBasReq", "LKAS11", 0),
      ("CF_Lkas_HbaSysState", "LKAS11", 0),
      ("CF_Lkas_FcwOpt", "LKAS11", 0),
      ("CF_Lkas_HbaOpt", "LKAS11", 0),
      ("CF_Lkas_FcwSysState", "LKAS11", 0),
      ("CF_Lkas_FcwCollisionWarning", "LKAS11", 0),
      ("CF_Lkas_MsgCount", "LKAS11", 0),  #  append
      ("CF_Lkas_FusionState", "LKAS11", 0),
      ("CF_Lkas_FcwOpt_USM", "LKAS11", 0),
      ("CF_Lkas_LdwsOpt_USM", "LKAS11", 0),

      ("HDA_USM", "LFAHDA_MFC", 0),
      ("HDA_Active", "LFAHDA_MFC", 0),
      ("HDA_Icon_State", "LFAHDA_MFC", 0),
      ("HDA_LdwSysState", "LFAHDA_MFC", 0),
      ("HDA_Icon_Wheel", "LFAHDA_MFC", 0),
    ]

    checks = [
      ("LKAS11", 100),
      ("LFAHDA_MFC", 20),      
    ]

    return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, 2)
