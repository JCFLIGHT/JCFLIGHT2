import codecs
import enum
import pathlib
import datetime
Import("env")


# 1.0 - INCREMENTE SEMPRE QUE HOUVER UM NOVO LANÇAMENTO
FIRMWARE_STORAGE_REVISION = 10

WAYPOINTS_MAXIMUM = 10  # NÚMERO MAXIMO DE WAYPOINTS SUPORTADO
OTHERS_PARAMS_MAXIMUM = 3  # ALTITUDE,TEMPO DO GPS-HOLD E O MODO DE VOO


class AddrSizeOf(enum.Enum):
    TYPE_8_BITS = 0x01
    TYPE_16_BITS = 0x02
    TYPE_32_BITS = 0x04
    TYPE_FLOAT = 0x04


class StorageSizeOf(enum.Enum):
    NONE = 0x00
    # 474 BYTES RESERVADOS DO ARMAZENAMENTO PARA O CLI
    CLI_SIZE_INITIAL_RESERVED = 0x01
    CLI_SIZE_FINAL_RESERVED = 0x1DB
    # 60 BYTES RESERVADOS DO ARMAZENAMENTO PARA AS CALIBRAÇÕES DOS SENSORES
    CALIBRATION_SIZE_INITIAL_RESERVED = 0x1E0
    CALIBRATION_SIZE_FINAL_RESERVED = 0x218
    # 455 BYTES RESERVADOS DO ARMAZENAMENTO PARA AS CONFIGURAÇÕES NORMAIS
    NORMAL_CONFIG_SIZE_INITIAL_RESERVED = 0x21C
    NORMAL_CONFIG_SIZE_FINAL_RESERVED = 0x3E3
    # 495 BYTES RESERVADOS DO ARMAZENAMENTO PARA AS CONFIGURAÇÕES DO MODO WAYPOINT
    WAYPOINT_SIZE_INITIAL_RESERVED = 0x3E8
    WAYPOINT_SIZE_FINAL_RESERVED = 0x5D7
    # ENDEREÇO PARA A VERIFICAÇÃO DE UPLOAD DO FIRMWARE
    FIRMWARE_RESERVED_MAGIC_ADDR = 0x5DC
    # 2000 BYTES RESERVADOS PARA USO
    TOTAL_SIZE_OF_STORAGE_RESERVED_TO_USE = 0x7D0


class PrintWithColors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


StorageLayout = [
    ['Grupo', 'Endereço Inicial', 'Endereço Final'],
    ['CLI_STORAGE', StorageSizeOf.CLI_SIZE_INITIAL_RESERVED.value,
        StorageSizeOf.CLI_SIZE_FINAL_RESERVED.value],
    ['NORMAL_CONFIG_STORAGE', StorageSizeOf.NORMAL_CONFIG_SIZE_INITIAL_RESERVED.value,
        StorageSizeOf.NORMAL_CONFIG_SIZE_FINAL_RESERVED.value],
    ['WAYPOINT_STORAGE', StorageSizeOf.WAYPOINT_SIZE_INITIAL_RESERVED.value,
        StorageSizeOf.WAYPOINT_SIZE_FINAL_RESERVED.value],
    ['FIRMWARE_MAGIC_ADDRESS', StorageSizeOf.NONE.value,
        StorageSizeOf.FIRMWARE_RESERVED_MAGIC_ADDR.value],
    ['TOTAL_SIZE_OF_STORAGE_RESERVED', StorageSizeOf.NONE.value,
        StorageSizeOf.TOTAL_SIZE_OF_STORAGE_RESERVED_TO_USE.value],
    ['SENSORS_CALIBRATION_STORAGE', StorageSizeOf.CALIBRATION_SIZE_INITIAL_RESERVED.value,
        StorageSizeOf.CALIBRATION_SIZE_FINAL_RESERVED.value],
]

DefsCLITable = [
    ['Nome da Definição', 'OffSet'],
    ['KP_ACC_AHRS_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['KI_ACC_AHRS_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['KP_MAG_AHRS_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['KI_MAG_AHRS_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['ANGLE_BLOCK_ARM_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['AL_AHRS_BA_ADDR', AddrSizeOf.TYPE_16_BITS.value],
    ['AL_IMU_GPS_VEL_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['AL_TRIGGER_MOTOR_DELAY_ADDR', AddrSizeOf.TYPE_16_BITS.value],
    ['AL_ELEVATOR_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['AL_SPINUP_ADDR', AddrSizeOf.TYPE_16_BITS.value],
    ['AL_SPINUP_TIME_ADDR', AddrSizeOf.TYPE_16_BITS.value],
    ['AL_MAX_THROTTLE_ADDR', AddrSizeOf.TYPE_16_BITS.value],
    ['AL_EXIT_ADDR', AddrSizeOf.TYPE_16_BITS.value],
    ['AL_ALTITUDE_ADDR', AddrSizeOf.TYPE_32_BITS.value],
    ['CC_BANKANGLE_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['CC_TIME_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['GIMBAL_MIN_ADDR', AddrSizeOf.TYPE_16_BITS.value],
    ['GIMBAL_MAX_ADDR', AddrSizeOf.TYPE_16_BITS.value],
    ['LAND_CHECKACC_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['AUTODISARM_ADDR', AddrSizeOf.TYPE_16_BITS.value],
    ['AUTODISARM_THR_MIN_ADDR', AddrSizeOf.TYPE_16_BITS.value],
    ['AUTODISARM_YPR_MIN_ADDR', AddrSizeOf.TYPE_16_BITS.value],
    ['AUTODISARM_YPR_MAX_ADDR', AddrSizeOf.TYPE_16_BITS.value],
    ['GPS_BAUDRATE_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['NAV_VEL_ADDR', AddrSizeOf.TYPE_16_BITS.value],
    ['WP_RADIUS_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['RTH_LAND_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['GPS_TILT_COMP_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['AIRSPEED_SAMPLES_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['ARM_TIME_SAFETY_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['DISARM_TIME_SAFETY_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['COMPASS_CAL_TIME_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['AS_AUTO_CAL_SCALE_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['CONT_SERVO_TRIM_ROTATION_LIMIT_ADDR', AddrSizeOf.TYPE_8_BITS.value],
]

DefsSensorsCalibrationTable = [
    ['Nome da Definição', 'OffSet'],
    ['ACC_ROLL_OFFSET_ADDR', AddrSizeOf.TYPE_16_BITS.value],
    ['ACC_PITCH_OFFSET_ADDR', AddrSizeOf.TYPE_16_BITS.value],
    ['ACC_YAW_OFFSET_ADDR', AddrSizeOf.TYPE_16_BITS.value],
    ['ACC_ROLL_SCALE_ADDR', AddrSizeOf.TYPE_16_BITS.value],
    ['ACC_PITCH_SCALE_ADDR', AddrSizeOf.TYPE_16_BITS.value],
    ['ACC_YAW_SCALE_ADDR', AddrSizeOf.TYPE_16_BITS.value],
    ['MAG_PITCH_OFFSET_ADDR', AddrSizeOf.TYPE_16_BITS.value],
    ['MAG_ROLL_OFFSET_ADDR', AddrSizeOf.TYPE_16_BITS.value],
    ['MAG_YAW_OFFSET_ADDR', AddrSizeOf.TYPE_16_BITS.value],
    ['MAG_ROLL_GAIN_ADDR', AddrSizeOf.TYPE_16_BITS.value],
    ['MAG_PITCH_GAIN_ADDR', AddrSizeOf.TYPE_16_BITS.value],
    ['MAG_YAW_GAIN_ADDR', AddrSizeOf.TYPE_16_BITS.value],
]

DefsNormalConfigTable = [
    ['Nome da Definição', 'OffSet'],
    ['SIMPLE_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['ALT_HOLD_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['GPS_HOLD_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['RTH_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['STABLIZE_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['ATTACK_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['PARACHUTE_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['AUTOFLIP_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['GIMBAL_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['FRAME_TYPE_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['RC_SEQUENCY_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['FF_OR_CD_ROLL_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['AUTOMISSION_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['AUTOLAND_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['ARMDISARM_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['DISP_PASSIVES_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['MAG_DECLINATION_ADDR', AddrSizeOf.TYPE_FLOAT.value],
    ['UART_NUMB_1_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['UART_NUMB_2_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['UART_NUMB_3_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['COMPASS_ROTATION_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['RTH_ALTITUDE_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['THROTTLE_DZ_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['YAW_DZ_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['PITCH_DZ_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['ROLL_DZ_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['KP_ROLL_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['KI_ROLL_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['KD_ROLL_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['KP_PITCH_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['KI_PITCH_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['KD_PITCH_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['KP_YAW_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['KI_YAW_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['KD_YAW_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['KP_VEL_Z_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['KI_VEL_Z_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['KD_VEL_Z_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['KP_GPSPOS_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['KI_GPSPOS_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['KP_POS_Z_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['KI_POS_Z_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['KD_POS_Z_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['KP_POS_RATE_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['KI_POS_RATE_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['KD_POS_RATE_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['KP_NAV_RATE_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['KI_NAV_RATE_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['KD_NAV_RATE_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['TPA_PERCENT_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['BREAKPOINT_ADDR', AddrSizeOf.TYPE_16_BITS.value],
    ['HW_GYRO_LPF_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['DERIVATIVE_LPF_ADDR', AddrSizeOf.TYPE_16_BITS.value],
    ['RC_LPF_ADDR', AddrSizeOf.TYPE_16_BITS.value],
    ['KALMAN_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['BI_ACC_LPF_ADDR', AddrSizeOf.TYPE_16_BITS.value],
    ['BI_GYRO_LPF_ADDR', AddrSizeOf.TYPE_16_BITS.value],
    ['BI_ACC_NOTCH_ADDR', AddrSizeOf.TYPE_16_BITS.value],
    ['BI_GYRO_NOTCH_ADDR', AddrSizeOf.TYPE_16_BITS.value],
    ['MOT_COMP_STATE_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['THR_ATTITUDE_MIN_ADDR', AddrSizeOf.TYPE_16_BITS.value],
    ['THR_ATTITUDE_MAX_ADDR', AddrSizeOf.TYPE_16_BITS.value],
    ['THROTTLE_MIDDLE_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['THROTTLE_EXPO_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['PR_RATE_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['PR_EXPO_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['YAW_RATE_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['YAW_EXPO_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['ROLL_BANK_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['PITCH_BANK_MIN_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['FF_OR_CD_PITCH_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['FF_OR_CD_YAW_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['SERVO1_RATE_ADDR', AddrSizeOf.TYPE_16_BITS.value],
    ['SERVO2_RATE_ADDR', AddrSizeOf.TYPE_16_BITS.value],
    ['SERVO3_RATE_ADDR', AddrSizeOf.TYPE_16_BITS.value],
    ['SERVO4_RATE_ADDR', AddrSizeOf.TYPE_16_BITS.value],
    ['FAILSAFE_VAL_ADDR', AddrSizeOf.TYPE_16_BITS.value],
    ['THROTTLE_MIN_ADDR', AddrSizeOf.TYPE_16_BITS.value],
    ['YAW_MIN_ADDR', AddrSizeOf.TYPE_16_BITS.value],
    ['PITCH_MIN_ADDR', AddrSizeOf.TYPE_16_BITS.value],
    ['ROLL_MIN_ADDR', AddrSizeOf.TYPE_16_BITS.value],
    ['THROTTLE_MAX_ADDR', AddrSizeOf.TYPE_16_BITS.value],
    ['YAW_MAX_ADDR', AddrSizeOf.TYPE_16_BITS.value],
    ['PITCH_MAX_ADDR', AddrSizeOf.TYPE_16_BITS.value],
    ['ROLL_MAX_ADDR', AddrSizeOf.TYPE_16_BITS.value],
    ['CH_REVERSE_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['SERVO1_MIN_ADDR', AddrSizeOf.TYPE_16_BITS.value],
    ['SERVO2_MIN_ADDR', AddrSizeOf.TYPE_16_BITS.value],
    ['SERVO3_MIN_ADDR', AddrSizeOf.TYPE_16_BITS.value],
    ['SERVO4_MIN_ADDR', AddrSizeOf.TYPE_16_BITS.value],
    ['SERVO1_MID_ADDR', AddrSizeOf.TYPE_16_BITS.value],
    ['SERVO2_MID_ADDR', AddrSizeOf.TYPE_16_BITS.value],
    ['SERVO3_MID_ADDR', AddrSizeOf.TYPE_16_BITS.value],
    ['SERVO4_MID_ADDR', AddrSizeOf.TYPE_16_BITS.value],
    ['SERVO1_MAX_ADDR', AddrSizeOf.TYPE_16_BITS.value],
    ['SERVO2_MAX_ADDR', AddrSizeOf.TYPE_16_BITS.value],
    ['SERVO3_MAX_ADDR', AddrSizeOf.TYPE_16_BITS.value],
    ['SERVO4_MAX_ADDR', AddrSizeOf.TYPE_16_BITS.value],
    ['SERVOS_REVERSE_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['SERVOS_LPF_ADDR', AddrSizeOf.TYPE_16_BITS.value],
    ['KP_AUTOLEVEL_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['KI_AUTOLEVEL_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['KP_HEADING_HOLD_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['HEADING_HOLD_RATE_LIMIT_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['PITCH_BANK_MAX_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['ATTACK_BANK_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['GPS_BANK_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['MAX_PITCH_LEVEL_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['MAX_ROLL_LEVEL_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['AIRSPEED_TYPE_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['INTEGRAL_RELAX_LPF_ADDR', AddrSizeOf.TYPE_16_BITS.value],
    ['KCD_OR_FF_LPF_ADDR', AddrSizeOf.TYPE_16_BITS.value],
    ['PITCH_LEVEL_TRIM_ADDR', AddrSizeOf.TYPE_16_BITS.value],
    ['BATT_VOLTAGE_FACTOR_ADDR', AddrSizeOf.TYPE_16_BITS.value],
    ['BATT_AMPS_VOLT_ADDR', AddrSizeOf.TYPE_16_BITS.value],
    ['BATT_AMPS_OFFSET_ADDR', AddrSizeOf.TYPE_16_BITS.value],
    ['BATT_MIN_VOLTAGE_ADDR', AddrSizeOf.TYPE_16_BITS.value],
    ['BATT_MAX_VOLTAGE_ADDR', AddrSizeOf.TYPE_16_BITS.value],
    ['BATT_NUMBER_OF_CELLS_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['BATT_CRIT_PERCENT_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['BATT_RTH_LOW_BATT_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['AUTO_PILOT_MODE_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['AIRSPEED_FACTOR_ADDR', AddrSizeOf.TYPE_32_BITS.value],
    ['INTEGRAL_WINDUP_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['CH_TUNNING_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['TUNNING_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['LAND_AFTER_RTH_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['HOVER_THROTTLE_ADDR', AddrSizeOf.TYPE_16_BITS.value],
    ['AIR_SPEED_REFERENCE_ADDR', AddrSizeOf.TYPE_16_BITS.value],
    ['TECS_PITCH2THR_FACTOR_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['TECS_PITCH2THR_LPF_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['TECS_AP_LPF_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['TECS_CRUISE_MIN_THR_ADDR', AddrSizeOf.TYPE_16_BITS.value],
    ['TECS_CRUISE_MAX_THR_ADDR', AddrSizeOf.TYPE_16_BITS.value],
    ['TECS_CRUISE_THR_ADDR', AddrSizeOf.TYPE_16_BITS.value],
    ['TECS_CIRCLE_DIR_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['CONT_SERVO_TRIM_STATE_ADDR', AddrSizeOf.TYPE_8_BITS.value],
]

FinalAddrOfWayPointCoordinates = (
    StorageLayout[3][1] + (WAYPOINTS_MAXIMUM * (AddrSizeOf.TYPE_32_BITS.value * 2)))
FinalAddrOfWayPointCoordinatesWithOffSet = (
    FinalAddrOfWayPointCoordinates + AddrSizeOf.TYPE_32_BITS.value)

DefsWayPointTable = [
    ['Nome da Definição', 'OffSet'],
    ['WAYPOINTS_MAXIMUM', WAYPOINTS_MAXIMUM],
    ['OTHERS_PARAMS_MAXIMUM', OTHERS_PARAMS_MAXIMUM],
    ['INITIAL_ADDR_OF_COORDINATES', StorageLayout[3][1]],
    ['FINAL_ADDR_OF_COORDINATES', FinalAddrOfWayPointCoordinates],
    ['INITIAL_ADDR_OF_OTHERS_PARAMS', FinalAddrOfWayPointCoordinatesWithOffSet],
    ['FINAL_ADDR_OF_OTHERS_PARAMS', StorageLayout[3][2]],
]


def Format_Entry(StrIn):

    return '%d' % round(StrIn)


def Generate_Defines(File, DefineName, StorageAddressOffSet):

    File.write('#define %s ' % DefineName)
    File.write(Format_Entry(StorageAddressOffSet))
    File.write("\n")


def Generate_Address_Type_To_Str(AddressSizeOf):

    StringRet = 'BYTES'

    if (AddressSizeOf == AddrSizeOf.TYPE_8_BITS.value):
        StringRet = 'BYTE'
    else:
        StringRet = 'BYTES'

    return StringRet


def Generate_WayPoint_Defs(File, InputTable):

    ColumnsCount = (len(InputTable) - 1)

    for TableSizeCount in range(ColumnsCount):
        File.write('#define %s ' % InputTable[TableSizeCount + 1][0])
        File.write(Format_Entry(InputTable[TableSizeCount + 1][1]))
        if(TableSizeCount == 0):
            File.write(' //NÚMERO MAXIMO DE WAYPOINTS SUPORTADO')
        if(TableSizeCount == 1):
            File.write(' //ALTITUDE,TEMPO DO GPS-HOLD E O MODO DE VOO')
        if(TableSizeCount > 1):
            print(f"{PrintWithColors.OKCYAN}DEF:{PrintWithColors.ENDC} %s" % InputTable[TableSizeCount + 1][0] + f"{PrintWithColors.OKCYAN}  ENDEREÇO DE ARMAZENAMENTO:{PrintWithColors.ENDC}" + '%d' %
                  InputTable[TableSizeCount + 1][1])
        else:
            print(f"{PrintWithColors.OKCYAN}DEF:{PrintWithColors.ENDC} %s" % InputTable[TableSizeCount + 1][0] + f"{PrintWithColors.OKCYAN}  VALOR:{PrintWithColors.ENDC}" + '%d' %
                  InputTable[TableSizeCount + 1][1])
        File.write("\n")


def Generate_Info_And_Defines(InputTable, InputStorageLayoutMin, InputStorageLayoutMax, InputErrorMessage, InputSuccessMessage):

    ColumnsCount = (len(InputTable) - 1)
    CheckSum = 0
    NextStorageAddress = 0
    PrevStorageAddress = 0
    SendMessageSuccess = False

    for TableSizeCount in range(ColumnsCount):
        CheckSum = CheckSum + InputTable[TableSizeCount + 1][1]

    for TableSizeCount in range(ColumnsCount):
        if(CheckSum >= InputStorageLayoutMax):
            print(InputErrorMessage)
            break
        SendMessageSuccess = True
        NextStorageAddress = NextStorageAddress + \
            InputTable[TableSizeCount + 1][1]
        Generate_Defines(
            File, InputTable[TableSizeCount + 1][0], PrevStorageAddress + 1 + InputStorageLayoutMin)
        print(f"{PrintWithColors.OKCYAN}DEF:{PrintWithColors.ENDC} %s" % InputTable[TableSizeCount + 1][0] + f"{PrintWithColors.OKCYAN}  ENDEREÇO DE ARMAZENAMENTO:{PrintWithColors.ENDC}%d" %
              (PrevStorageAddress + 1 + InputStorageLayoutMin) + f"{PrintWithColors.OKCYAN}   TAMANHO:{PrintWithColors.ENDC}%d" % InputTable[TableSizeCount + 1][1] + ' %s' % Generate_Address_Type_To_Str(InputTable[TableSizeCount + 1][1]))
        PrevStorageAddress = NextStorageAddress

    if (SendMessageSuccess):
        print(InputSuccessMessage)


def Generate_Code(File, Date):
    # GERA O TOPO DA EXTENSÃO
    File.write("\
/* \n\
   Este arquivo faz parte da JCFLIGHT.\
   \n\
   JCFLIGHT é um software livre: você pode redistribuí-lo e/ou modificar \n\
   sob os termos da GNU General Public License conforme publicada por \n\
   a Free Software Foundation, seja a versão 3 da Licença, ou \n\
   (à sua escolha) qualquer versão posterior. \n\
   \n\
   JCFLIGHT é distribuído na esperança de ser útil, \n\
   mas SEM QUALQUER GARANTIA; sem mesmo a garantia implícita de \n\
   COMERCIALIZAÇÃO ou ADEQUAÇÃO A UM DETERMINADO FIM. Veja o \n\
   GNU General Public License para mais detalhes. \n\
   \n\
   Você deve ter recebido uma cópia da Licença Pública Geral GNU \n\
   junto com a JCFLIGHT. Caso contrário, consulte <http://www.gnu.org/licenses/>. \n\
*/\
\n\n")

    File.write('#pragma once\n\n')
    File.write('/*\n')
    File.write('BAR - BASE ADDRESS REGISTER\n\n')
    File.write(
        'ESSE ARQUIVO HEADER É GERADO AUTOMATICAMENTE SEMPRE QUE HOUVER UMA NOVA COMPILAÇÃO - POR FAVOR,NÃO O EDITE MANUALMENTE!\n\n')
    File.write('ATUALIZADO EM %s\n' % Date)
    File.write('*/\n\n')

    File.write('//INCREMENTE SEMPRE QUE HOUVER UM NOVO LANÇAMENTO\n')
    File.write('#define FIRMWARE_STORAGE_REVISION' + ' %d' %
               FIRMWARE_STORAGE_REVISION + ' //%.1f' %
               (FIRMWARE_STORAGE_REVISION / 10) + '\n\n')

    print('\n')

    print(f"{PrintWithColors.OKBLUE}FIRMWARE_STORAGE_REVISION:{PrintWithColors.ENDC}" + ' %.1f' %
          (FIRMWARE_STORAGE_REVISION / 10))

    File.write('//NÚMERO DE BYTES DO ARMAZENAMENTO RESERVADOS PARA USO\n')
    File.write('#define ' + '%s' %
               StorageLayout[5][0] + ' %d' % StorageLayout[5][2] + '\n\n')

    print(f"{PrintWithColors.OKBLUE}%s:{PrintWithColors.ENDC}" %
          StorageLayout[5][0] + ' %d' % StorageLayout[5][2])

    # ENDEREÇO DO PRIMEIRO UPLOAD
    File.write(
        '//ENDEREÇO PARA VERIFICAR SE É O PRIMEIRO UPLOAD DA VERSÃO DO FIRMWARE\n')
    File.write('#define ' + '%s' %
               StorageLayout[4][0] + ' %d' % StorageLayout[4][2] + '\n\n')

    print(f"{PrintWithColors.OKBLUE}%s:{PrintWithColors.ENDC}" %
          StorageLayout[4][0] + ' %d' % StorageLayout[4][2])

    print('\n')

    print(f"{PrintWithColors.HEADER}-----------------------------------------------------------DEFINIÇÕES DO CLI------------------------------------------------------------{PrintWithColors.ENDC}")

    File.write('//ENDEREÇOS PARA O CLI\n')
    Generate_Info_And_Defines(
        DefsCLITable, StorageLayout[1][1], StorageLayout[1][2], f"{PrintWithColors.WARNING}!!!FALHA!!! OS ENDEREÇOS DO CLI ATINGIRAM O NÚMERO MAXIMO DE ENDEREÇOS DISPONIVEIS{PrintWithColors.ENDC}", f"{PrintWithColors.OKGREEN}OS ENDEREÇOS DO CLI FORAM GERADOS COM SUCESSO!{PrintWithColors.ENDC}")

    print(f"{PrintWithColors.HEADER}-------------------------------------------------------------------------------------------------------------------------------------------{PrintWithColors.ENDC}")

    print('\n')

    print(f"{PrintWithColors.HEADER}--------------------------------------------------------DEFINIÇÕES DAS CALIBRAÇÕES---------------------------------------------------------{PrintWithColors.ENDC}")

    File.write('\n//ENDEREÇOS PARA AS CALIBRAÇÕES DOS SENSORES\n')
    Generate_Info_And_Defines(
        DefsSensorsCalibrationTable, StorageLayout[6][1], StorageLayout[6][2], f"{PrintWithColors.WARNING}!!!FALHA!!! OS ENDEREÇOS DAS CALIBRAÇÕES ATINGIRAM O NÚMERO MAXIMO DE ENDEREÇOS DISPONIVEIS{PrintWithColors.ENDC}", f"{PrintWithColors.OKGREEN}OS ENDEREÇOS DAS CALIBRAÇÕES FORAM GERADOS COM SUCESSO!{PrintWithColors.ENDC}")

    print(f"{PrintWithColors.HEADER}-------------------------------------------------------------------------------------------------------------------------------------------{PrintWithColors.ENDC}")

    print('\n')

    print(f"{PrintWithColors.HEADER}-------------------------------------------------------DEFINIÇÕES DAS CONFIGURAÇÕES--------------------------------------------------------{PrintWithColors.ENDC}")

    File.write('\n//ENDEREÇOS PARA AS CONFIGURAÇÕES\n')
    Generate_Info_And_Defines(
        DefsNormalConfigTable, StorageLayout[2][1], StorageLayout[2][2], f"{PrintWithColors.WARNING}!!!FALHA!!! OS ENDEREÇOS DAS CONFIGURAÇÕES NORMAIS ATINGIRAM O NÚMERO MAXIMO DE ENDEREÇOS DISPONIVEIS{PrintWithColors.ENDC}", f"{PrintWithColors.OKGREEN}OS ENDEREÇOS DAS CONFIGURAÇÕES FORAM GERADOS COM SUCESSO!{PrintWithColors.ENDC}")

    print(f"{PrintWithColors.HEADER}-------------------------------------------------------------------------------------------------------------------------------------------{PrintWithColors.ENDC}")

    print('\n')

    print(f"{PrintWithColors.HEADER}--------------------------------------------------------DEFINIÇÕES DO MODO WAYPOINT--------------------------------------------------------{PrintWithColors.ENDC}")

    File.write('\n//CONFIGURAÇÕES E ENDEREÇOS PARA O MODO WAYPOINT\n')
    Generate_WayPoint_Defs(File, DefsWayPointTable)

    print(f"{PrintWithColors.HEADER}------------------------------------------------------------------------------------------------------------------------------------------{PrintWithColors.ENDC}")

    print('\n')


env.Dump()
try:

    DateAndTime = str(datetime.datetime.now())
    Year = DateAndTime[0:4]
    Day = DateAndTime[5:7]
    Month = DateAndTime[8:10]
    Hours = DateAndTime[11:13]
    Minutes = DateAndTime[14:16]
    Seconds = DateAndTime[17:19]

except:
    print('')

with codecs.open(pathlib.PurePath('__main__').parent / 'src' / 'BAR' / 'BAR.h', "w", "utf-8-sig") as File:
    Generate_Code(File, "{}/{}/{} ÁS {}:{}:{}".format(Month,
                  Day, Year, Hours, Minutes, Seconds))
