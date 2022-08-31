# Use this class to control the camera.

# Imports
import sys
import os
import time
import cv2
import threading
import numpy as np
import glob
import logging
from .ImageConvert import *
import arducam_config_parser
import ArducamSDK



# Class definition
class Snapper:

    # Initialization
    def __init__(self, logger):
        self.running = True
        self.cfg = {}
        self.handle = {}
        self.logger = logger
        self.save_snaps = "/usr/share/oresat-star-tracker/data/snaps/"
        self.save_solves = "/usr/share/oresat-star-tracker/data/solves/"
        self.CONFIG_FILE_NAME = '/home/debian/arducam/ArduCAM_USB_Camera_Shield/Config/USB2.0_UC-391_Rev.D/DVP/AR0134/AR0134_MONO_8b_1280x964_31fps.cfg'

        if self.camera_initFromFile(self.CONFIG_FILE_NAME):
            #logger.info("starting camera thread")
            ArducamSDK.Py_ArduCam_setMode(self.handle,ArducamSDK.EXTERNAL_TRIGGER_MODE)
            logger.info("started camera thread")
            #print("Started")
        else:
            print("could not open ;(")
            #self.logger.info("Could not start camera :(")

    def configBoard(self, config):
        ArducamSDK.Py_ArduCam_setboardConfig(self.handle, config.params[0], \
        config.params[1], config.params[2], config.params[3], \
        config.params[4:config.params_length])


    def camera_initFromFile(self, fileName):
        #load config file
        config = arducam_config_parser.LoadConfigFile(fileName)

        camera_parameter = config.camera_param.getdict()
        Width = camera_parameter["WIDTH"]
        Height = camera_parameter["HEIGHT"]

        BitWidth = camera_parameter["BIT_WIDTH"]
        ByteLength = 1
        if BitWidth > 8 and BitWidth <= 16:
            ByteLength = 2
        FmtMode = camera_parameter["FORMAT"][0]
        self.color_mode = camera_parameter["FORMAT"][1]
        print("color mode",self.color_mode)

        I2CMode = camera_parameter["I2C_MODE"]
        I2cAddr = camera_parameter["I2C_ADDR"]
        TransLvl = camera_parameter["TRANS_LVL"]
        self.cfg = {"u32CameraType":0x00,
                "u32Width":Width,"u32Height":Height,
                "usbType":0,
                "u8PixelBytes":ByteLength,
                "u16Vid":0,
                "u32Size":0,
                "u8PixelBits":BitWidth,
                "u32I2cAddr":I2cAddr,
                "emI2cMode":I2CMode,
                "emImageFmtMode":FmtMode,
                "u32TransLvl":TransLvl }


        #ret,handle,rtn_cfg = ArducamSDK.Py_ArduCam_open(cfg,0)
        #devices_num,index,serials = ArducamSDK.Py_ArduCam_scan()
        ret,self.handle,rtn_cfg = ArducamSDK.Py_ArduCam_autoopen(self.cfg)
        if ret == 0:

            #ArducamSDK.Py_ArduCam_writeReg_8_8(handle,0x46,3,0x00)
            usb_version = rtn_cfg['usbType']
            configs = config.configs
            configs_length = config.configs_length
            for i in range(configs_length):
                type = configs[i].type
                if ((type >> 16) & 0xFF) != 0 and ((type >> 16) & 0xFF) != usb_version:
                    continue
                if type & 0xFFFF == arducam_config_parser.CONFIG_TYPE_REG:
                    ArducamSDK.Py_ArduCam_writeSensorReg(self.handle, configs[i].params[0], configs[i].params[1])
                elif type & 0xFFFF == arducam_config_parser.CONFIG_TYPE_DELAY:
                    time.sleep(float(configs[i].params[0])/1000)
                elif type & 0xFFFF == arducam_config_parser.CONFIG_TYPE_VRCMD:
                    self.configBoard(configs[i])

            ArducamSDK.Py_ArduCam_registerCtrls(self.handle, config.controls, config.controls_length)
            ArducamSDK.Py_ArduCam_setCtrl(self.handle, "setFramerate", 30)
            self.set_exposure(0.1)
            self.set_gain(150)
            # ArducamSDK.Py_ArduCam_setCtrl(handle, "setExposure", 5)
            # ArducamSDK.Py_ArduCam_setCtrl(handle, "setExposureTime", 33000)
            # ArducamSDK.Py_ArduCam_setCtrl(handle, "setGain", 5)
            # ArducamSDK.Py_ArduCam_setCtrl(handle, "setAnalogueGain", 100)

            rtn_val,datas = ArducamSDK.Py_ArduCam_readUserData(self.handle,0x400-16, 16)
            print("Serial: %c%c%c%c-%c%c%c%c-%c%c%c%c"%(datas[0],datas[1],datas[2],datas[3],
                                                        datas[4],datas[5],datas[6],datas[7],
                                                        datas[8],datas[9],datas[10],datas[11]))

            return True
        else:
            print("open fail,rtn_val = ",ret)
            return False 


    # Start the camera / PRU
    def start(self):
        if not self.running:
            if self.camera_initFromFile(self.CONFIG_FILE_NAME):
                ArducamSDK.Py_ArduCam_setMode(self.handle,ArducamSDK.EXTERNAL_TRIGGER_MODE)
                self.running = True
                #print("started camera")
                self.logger.info("Started camera")
        else:
            self.logger.info("Camera already running")
            #print('camera already running')

    # Stop the camera / PRU
    def stop(self):
        if self.running:
            rtn_val = ArducamSDK.Py_ArduCam_close(self.handle)
            if rtn_val == 0:
                #print("Stopped camera success")
                self.running = False
                self.logger.info("Stopped camera success!")
            else:
                #print("stopped camera fail")
                self.loggrer.info("Stopped camera fail!")
        else:
            #print("camera already stopped")
            self.logger.info("Camera already stopped.")

    # Restart the camera / PRU
    def restart(self):
        if self.running:
            self.stop()
            time.sleep(1)
            self.start()
        else:
            self.start()
        #print("restarted camera")
        self.logger.info("Restarted camera.")

    # Capture a photo as a snap
    def capture_snap(self, curr_num):
        ArducamSDK.Py_ArduCam_softTrigger(self.handle)
        rtn_val,data,rtn_cfg = ArducamSDK.Py_ArduCam_getSingleFrame(self.handle)
        datasize = rtn_cfg['u32Size']

        if rtn_val != 0 or datasize == 0:
            ArducamSDK.Py_ArduCam_del(self.handle)
            self.logger.info("read data fail!")
            #print("read data fail")
            return 0

        image = convert_image(data, rtn_cfg,self.color_mode)
        image = cv2.resize(image, (640, 480))

        date_and_time = time.strftime("%d-%m-%Y_%H-%M-%S", time.gmtime())
        path = self.save_snaps + f"{curr_num}_{date_and_time}.jpg"

        cv2.imwrite(path,image)

        if curr_num >= 50:
            file_to_remove = glob.glob(self.save_snaps + f"{curr_num - 50}_*.jpg")[0]
            os.remove(file_to_remove)

        #print("Saved snap to ", path)
        self.logger.info(f"Saved snap to {path}.")
        return path



    def capture_solve(self, curr_num):
        ArducamSDK.Py_ArduCam_softTrigger(self.handle)
        rtn_val,data,rtn_cfg = ArducamSDK.Py_ArduCam_getSingleFrame(self.handle)
        datasize = rtn_cfg['u32Size']

        if rtn_val != 0 or datasize == 0:
            ArducamSDK.Py_ArduCam_del(self.handle)
            self.logger.info("read data fail!")
            #print("read data fail")
            return 0

        image = convert_image(data, rtn_cfg,self.color_mode)
        image = cv2.resize(image, (640,480))
        date_and_time = time.strftime("%d-%m-%Y_%H-%M-%S", time.gmtime())
        path = self.save_solves + f"{curr_num}_{date_and_time}.jpg"
        cv2.imwrite(path,image)
        if curr_num >= 50:
            file_to_remove = glob.glob(self.save_solves + f"{curr_num - 50}_*.jpg")[0]
            os.remove(file_to_remove)

        #print("Saved solve to ", path)

        return path


    def set_exposure(self, exposure):
        row_time = 1650.0/87000000.0
        ArducamSDK.Py_ArduCam_writeSensorReg(self.handle, 0x3012, int(exposure/row_time))
        return

    def set_gain(self, gain):
        ArducamSDK.Py_ArduCam_writeSensorReg(self.handle, 0x305E, gain)
        return
