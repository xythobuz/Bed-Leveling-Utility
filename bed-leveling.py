#!/usr/bin/env python

#
# bed-leveling.py
#
# Written in June 2016 by:
# Thomas Buck <xythobuz@xythobuz.de>
#
# Manual Mesh Bed Leveling GUI utility for 3D printers with Marlin Firmware.
# At the time of me writing this program, Marlin had a bug that caused the
# manual mesh bed leveling process to not work using the connected LCD display,
# leaving G-Codes sent over the serial port as the only option. Because manually
# inserting these in a terminal program proved to be very tedious, and none of the
# G-Code senders I've tested had the features I wanted (mainly easily configurable
# step sizes), I've wrote this simple little program.
#
# After starting the program, select the serial port connected to your printer,
# the correct baudrate for communicating with it and press Connect.
# The X, Y and Z value displayed are periodically polled from the printer and
# are not very accurate. Zc is what is manipulated using the up- and down-arrow
# buttons or keys. The value shown there will be more accurate.
#
# Before starting with the leveling procedure, the current Mesh info has to be
# polled from the printer. This will happen automatically after connecting.
# When ready, press Start, then adjust the Z-height for each measuring point
# and press Next or the right-arrow key to move on. After you're finished,
# you should store the new mesh data in the EEPROM using the Save button.
#
# Dependencies:
#  - pySerial (>= v3.0)
#  - wxPython
#
# This has been developed and tested on a Mac OS X 10.10.5 machine
# with Python 2.7 and dependencies installed with MacPorts,
# connected to a modified Fabrikator Mini V1.5 with Marlin 1.1.0-RC6.
#

import wx
import serial
import serial.threaded
import serial.tools.list_ports
import re

POLL_SERIAL_PORTS_INTERVALL = 4000
POLL_COORDINATES_INTERVALL = 2500
POLL_TEMPERATURE_INTERVALL = 3000
MESH_DATA_RETRY_TIMEOUT = 2000
BUSY_PROCESSING_STEPS = 6;
AVAILABLE_BAUD_RATES = ["2400", "9600", "19200", "38400", "57600", "115200", "250000"]
DEFAULT_BAUD_RATE = "115200"
DEFAULT_SERIAL_PORTS = ["/dev/cu.usbserial-AI02LQH7", "/dev/cu.SLAB_USBtoUART"]

GCODE_EEPROM_SAVE = "M500"
GCODE_STOP_IDLE_HOLD = "M84"
GCODE_MOVE_TO_ORIGIN = "G28"
GCODE_GET_CURRENT_POSITION = "M114"
GCODE_MESH_INFO = "G29 S0"
GCODE_MESH_START = "G29 S1"
GCODE_MESH_NEXT = "G29 S2"
GCODE_MOVE_Z = "G1 Z{}"
GCODE_SET_BED_TEMP = "M140 S{}"
GCODE_GET_TEMP = "M105"

PRINTER_RESPONSE_OK = "ok"
PRINTER_RESPONSE_BUSY = "echo:busy: processing"
PRINTER_REGEX_COORDINATES = "^X:\d+\.\d+ Y:\d+\.\d+ Z:\d+\.\d+ E:\d+\.\d+ Count X:\s*\d+ Y:\s*\d+ Z:\s*\d+$"
PRINTER_REGEX_MESH_POINTS = "^Num X,Y: \d,\d$"
PRINTER_REGEX_MESH_Z = "^Z search height: \d\.*\d*$"
PRINTER_REGEX_TEMP = "^ok T:\d\d*\.*\d* \/\d\d*\.*\d* B:\d\d*\.*\d* \/\d\d*\.*\d* B@:\d\d*\.*\d* @:\d\d*\.*\d*$"

# Event for sending new XYZ coordinates from the serial thread to the GUI thread
myEVT_COORDINATES = wx.NewEventType()
EVT_COORDINATES = wx.PyEventBinder(myEVT_COORDINATES, 1)
class CoordinatesEvent(wx.PyCommandEvent):
    def __init__(self, etype, eid, value = None):
        wx.PyCommandEvent.__init__(self, etype, eid)
        self.value = value

    def GetValue(self):
        return self.value

# Event for sending new mesh point info
myEVT_MESH_POINTS = wx.NewEventType()
EVT_MESH_POINTS = wx.PyEventBinder(myEVT_MESH_POINTS, 1)
class MeshPointEvent(wx.PyCommandEvent):
    def __init__(self, etype, eid, x = None, y = None):
        wx.PyCommandEvent.__init__(self, etype, eid)
        self.x = x
        self.y = y

    def GetX(self):
        return self.x

    def GetY(self):
        return self.y

# Event for sending new mesh point info
myEVT_MESH_Z = wx.NewEventType()
EVT_MESH_Z = wx.PyEventBinder(myEVT_MESH_Z, 1)
class MeshZEvent(wx.PyCommandEvent):
    def __init__(self, etype, eid, z = None):
        wx.PyCommandEvent.__init__(self, etype, eid)
        self.z = z

    def GetZ(self):
        return self.z

# Event for sending temperature info
myEVT_TEMPERATURE = wx.NewEventType()
EVT_TEMPERATURE = wx.PyEventBinder(myEVT_TEMPERATURE, 1)
class TemperatureEvent(wx.PyCommandEvent):
    def __init__(self, etype, eid, bed = None):
        wx.PyCommandEvent.__init__(self, etype, eid)
        self.bed = bed

    def GetBed(self):
        return self.bed

# Event for sending a new busy indicator status update
myEVT_STATUS = wx.NewEventType()
EVT_STATUS = wx.PyEventBinder(myEVT_STATUS, 1)
class StatusEvent(wx.PyCommandEvent):
    def __init__(self, etype, eid, value = None):
        wx.PyCommandEvent.__init__(self, etype, eid)
        self.value = value

    def GetValue(self):
        return self.value

# Serial Thread reading from and writing to the printer, line-based
class GCodeReader(serial.threaded.LineReader):

    TERMINATOR = b'\n'

    regexCoordinates = re.compile(PRINTER_REGEX_COORDINATES, re.M)
    regexMeshPoints = re.compile(PRINTER_REGEX_MESH_POINTS, re.M)
    regexMeshZ = re.compile(PRINTER_REGEX_MESH_Z, re.M)
    regexTemp = re.compile(PRINTER_REGEX_TEMP, re.M)

    def setParent(self, parent):
        self.parent = parent

    def handle_line(self, data):
        if data == PRINTER_RESPONSE_OK:
            event = StatusEvent(myEVT_STATUS, -1, 0)
            wx.PostEvent(self.parent, event)
            return

        if data == PRINTER_RESPONSE_BUSY:
            event = StatusEvent(myEVT_STATUS, -1, 1)
            wx.PostEvent(self.parent, event)
            return

        if self.regexCoordinates.match(data):
            afterX = data.split("X:")[1]
            x = afterX.split(" ")[0]
            afterY = data.split("Y:")[1]
            y = afterY.split(" ")[0]
            afterZ = data.split("Z:")[1]
            z = afterZ.split(" ")[0]
            str = "X: {} Y: {} Z: {}".format(x, y, z)
            event = CoordinatesEvent(myEVT_COORDINATES, -1, str)
            wx.PostEvent(self.parent, event)
            return

        if self.regexMeshPoints.match(data):
            firstDigit = data.split("X,Y: ")[1]
            x = firstDigit.split(",")[0]
            y = firstDigit.split(",")[1]
            event = MeshPointEvent(myEVT_MESH_POINTS, -1, x, y)
            wx.PostEvent(self.parent, event)
            return

        if self.regexMeshZ.match(data):
            num = data.split("Z search height: ")[1]
            event = MeshZEvent(myEVT_MESH_Z, -1, float(num))
            wx.PostEvent(self.parent, event)
            return

        if self.regexTemp.match(data):
            bed = data.split(" B:")[1]
            temp = bed.split(" /")[0]
            event = TemperatureEvent(myEVT_TEMPERATURE, -1, float(temp))
            wx.PostEvent(self.parent, event)
            return

# Main Window class
class WizardFrame(wx.Frame):
    def __init__(self, *args, **kwargs):
        wx.Frame.__init__(self, *args, **kwargs)

        self.hasQuit = False
        self.isConnected = False
        self.serial = serial.Serial()
        self.thread = serial.threaded.ReaderThread(self.serial, GCodeReader)
        self.isLeveling = False
        self.currentPoint = 0
        self.meshPoints = -1
        self.step = 0.025
        self.currentZ = 0.0
        self.startZ = 0.0

        self.Bind(EVT_COORDINATES, self.OnCoordinates)
        self.Bind(EVT_STATUS, self.OnStatus)
        self.Bind(EVT_MESH_POINTS, self.OnMeshPoints)
        self.Bind(EVT_MESH_Z, self.OnMeshZ)
        self.Bind(EVT_TEMPERATURE, self.OnTemperature)

        # Menubar and Items
        MenuBar = wx.MenuBar()
        FileMenu = wx.Menu()
        MenuBar.Append(FileMenu, "&File")
        self.SetMenuBar(MenuBar)
        item = FileMenu.Append(wx.ID_EXIT, text = "&Exit")
        self.Bind(wx.EVT_MENU, self.OnQuit, item)
        self.Bind(wx.EVT_CLOSE, self.OnQuit)
        self.Bind(wx.EVT_KEY_DOWN, self.OnKeyDown)

        outerPanelSizer = wx.BoxSizer(wx.VERTICAL)

        # Panel for serial control widgets
        self.panelSerial = wx.Panel(self)
        panelSerialSizer = wx.BoxSizer(wx.HORIZONTAL)
        self.panelSerial.SetSizer(panelSerialSizer)
        outerPanelSizer.Add(self.panelSerial, 0, wx.TOP | wx.EXPAND, 4)
        self.panelSerial.Bind(wx.EVT_KEY_DOWN, self.OnKeyDown)

        # List all available serial ports
        serialPortList = serial.tools.list_ports.comports()
        serialPortNames = []
        for port in serialPortList:
            serialPortNames.append(port[0])

        # Combobox for serial ports
        self.comboBoxSerial = wx.ComboBox(parent = self.panelSerial, choices = serialPortNames, style = wx.CB_READONLY)
        self.comboBoxSerial.SetMinSize((200, -1))
        panelSerialSizer.Add(self.comboBoxSerial, 1, wx.LEFT, 5)
        self.comboBoxSerial.Bind(wx.EVT_KEY_DOWN, self.OnKeyDown)

        # Select default serial port(s)
        for port in serialPortList:
            if port[0] in DEFAULT_SERIAL_PORTS:
                self.comboBoxSerial.SetStringSelection(port[0])

        # Regularly update serial port list
        wx.FutureCall(POLL_SERIAL_PORTS_INTERVALL, self.EnumerateSerialPorts)

        # Combobox for baudrates
        self.comboBoxBaud = wx.ComboBox(parent = self.panelSerial, choices = AVAILABLE_BAUD_RATES, style = wx.CB_READONLY)
        self.comboBoxBaud.SetMinSize((80, -1))
        panelSerialSizer.Add(self.comboBoxBaud, 0)
        self.comboBoxBaud.Bind(wx.EVT_KEY_DOWN, self.OnKeyDown)

        for baud in AVAILABLE_BAUD_RATES:
            if baud == DEFAULT_BAUD_RATE:
                self.comboBoxBaud.SetStringSelection(DEFAULT_BAUD_RATE)

        # Button for connecting and disconnecting
        self.buttonSerial = wx.Button(parent = self.panelSerial, label = "Connect")
        self.buttonSerial.Bind(wx.EVT_BUTTON, self.OnConnectDisconnect)
        panelSerialSizer.Add(self.buttonSerial, 0, wx.TOP, 2)
        panelSerialSizer.AddSpacer(5)
        self.buttonSerial.Bind(wx.EVT_KEY_DOWN, self.OnKeyDown)

        # Panel for status labels
        self.panelStatus = wx.Panel(self)
        panelStatusSizer = wx.BoxSizer(wx.HORIZONTAL)
        self.panelStatus.SetSizer(panelStatusSizer)
        outerPanelSizer.Add(self.panelStatus, 0, wx.EXPAND | wx.TOP, 3)
        self.panelStatus.Bind(wx.EVT_KEY_DOWN, self.OnKeyDown)

        # Labels for coordinates
        self.labelX = wx.StaticText(parent = self.panelStatus, label = "X: ??.??")
        panelStatusSizer.Add(self.labelX, 0, wx.LEFT, 5)
        panelStatusSizer.AddStretchSpacer(1)
        self.labelX.Bind(wx.EVT_KEY_DOWN, self.OnKeyDown)

        self.labelY = wx.StaticText(parent = self.panelStatus, label = "Y: ??.??")
        panelStatusSizer.Add(self.labelY, 0)
        panelStatusSizer.AddStretchSpacer(1)
        self.labelY.Bind(wx.EVT_KEY_DOWN, self.OnKeyDown)

        self.labelZ = wx.StaticText(parent = self.panelStatus, label = "Z: ??.??")
        panelStatusSizer.Add(self.labelZ, 0, wx.RIGHT, 5)
        panelStatusSizer.AddStretchSpacer(1)
        self.labelZ.Bind(wx.EVT_KEY_DOWN, self.OnKeyDown)

        self.labelZc = wx.StaticText(parent = self.panelStatus, label = "Zc: ??.???")
        panelStatusSizer.Add(self.labelZc, 0, wx.RIGHT, 5)
        self.labelZc.Bind(wx.EVT_KEY_DOWN, self.OnKeyDown)

        # Panel for general actions
        self.panelAction = wx.Panel(self)
        panelActionSizer = wx.BoxSizer(wx.HORIZONTAL)
        self.panelAction.SetSizer(panelActionSizer)
        outerPanelSizer.Add(self.panelAction, 0, wx.EXPAND | wx.TOP, 5)
        self.panelAction.Bind(wx.EVT_KEY_DOWN, self.OnKeyDown)

        # Button for homing
        self.buttonHome = wx.Button(parent = self.panelAction, label = "Home")
        self.buttonHome.Bind(wx.EVT_BUTTON, self.OnHome)
        self.buttonHome.Enable(False)
        panelActionSizer.AddSpacer(5)
        panelActionSizer.Add(self.buttonHome, 0, wx.BOTTOM, 1)
        self.buttonHome.Bind(wx.EVT_KEY_DOWN, self.OnKeyDown)

        # Button for steppers off
        self.buttonOff = wx.Button(parent = self.panelAction, label = "Off")
        self.buttonOff.Bind(wx.EVT_BUTTON, self.OnStepperOff)
        self.buttonOff.Enable(False)
        panelActionSizer.AddSpacer(5)
        panelActionSizer.Add(self.buttonOff, 0, wx.BOTTOM, 1)
        self.buttonOff.Bind(wx.EVT_KEY_DOWN, self.OnKeyDown)

        # Status Gauge
        self.gauge = wx.Gauge(parent = self.panelAction)
        self.gauge.SetRange(BUSY_PROCESSING_STEPS)
        panelActionSizer.Add(self.gauge, 1, wx.LEFT | wx.RIGHT, 5)
        self.gauge.Bind(wx.EVT_KEY_DOWN, self.OnKeyDown)

        # Bottom panel
        self.panelBottom = wx.Panel(self)
        panelBottomSizer = wx.BoxSizer(wx.HORIZONTAL)
        self.panelBottom.SetSizer(panelBottomSizer)
        outerPanelSizer.Add(self.panelBottom, 0, wx.EXPAND | wx.TOP, 5)
        self.panelBottom.Bind(wx.EVT_KEY_DOWN, self.OnKeyDown)

        # Config panel
        self.panelConfig = wx.Panel(self.panelBottom)
        panelConfigSizer = wx.BoxSizer(wx.VERTICAL)
        self.panelConfig.SetSizer(panelConfigSizer)
        panelBottomSizer.Add(self.panelConfig, 2, wx.EXPAND | wx.LEFT, 5)
        self.panelConfig.Bind(wx.EVT_KEY_DOWN, self.OnKeyDown)

        # Step Size Label
        stepSizeLabel = wx.StaticText(self.panelConfig, label="Step:")
        panelConfigSizer.Add(stepSizeLabel, 0, wx.LEFT, 5)
        stepSizeLabel.Bind(wx.EVT_KEY_DOWN, self.OnKeyDown)

        # Step Size Text Input
        self.stepSize = wx.TextCtrl(parent = self.panelConfig, value="0.025", style = wx.TE_DONTWRAP | wx.TE_PROCESS_ENTER)
        self.stepSize.SetMinSize((65, -1))
        self.stepSize.Enable(False)
        self.stepSize.Bind(wx.EVT_TEXT_ENTER, self.OnNewStepSize)
        panelConfigSizer.Add(self.stepSize, 0, wx.LEFT | wx.TOP, 5)

        # Bed Temperature Label
        self.bedTemperatureLabel = wx.StaticText(self.panelConfig, label="Bed: ??.?")
        panelConfigSizer.Add(self.bedTemperatureLabel, 0, wx.LEFT | wx.TOP, 5)
        self.bedTemperatureLabel.Bind(wx.EVT_KEY_DOWN, self.OnKeyDown)

        # Bed Temperature Text Input
        self.bedTemperature = wx.TextCtrl(parent = self.panelConfig, value="0", style = wx.TE_DONTWRAP | wx.TE_PROCESS_ENTER)
        self.bedTemperature.SetMinSize((65, -1))
        self.bedTemperature.Enable(False)
        self.bedTemperature.Bind(wx.EVT_TEXT_ENTER, self.OnNewBedTemperature)
        panelConfigSizer.Add(self.bedTemperature, 0, wx.LEFT | wx.TOP | wx.BOTTOM, 5)

        # Control-Buttons Panel
        self.panelControl = wx.Panel(self.panelBottom)
        panelControlSizer = wx.BoxSizer(wx.VERTICAL)
        self.panelControl.SetSizer(panelControlSizer)
        panelBottomSizer.Add(self.panelControl, 2, wx.EXPAND | wx.LEFT, 5)
        self.panelControl.Bind(wx.EVT_KEY_DOWN, self.OnKeyDown)

        # Up Button
        self.buttonUp = wx.Button(parent = self.panelControl, label = "/\\")
        self.buttonUp.Bind(wx.EVT_BUTTON, self.OnUp)
        self.buttonUp.Enable(False)
        panelControlSizer.AddStretchSpacer(1)
        panelControlSizer.Add(self.buttonUp, 2, wx.ALIGN_CENTER_HORIZONTAL)
        self.buttonUp.Bind(wx.EVT_KEY_DOWN, self.OnKeyDown)

        # Down Button
        self.buttonDown = wx.Button(parent = self.panelControl, label = "\\/")
        self.buttonDown.Bind(wx.EVT_BUTTON, self.OnDown)
        self.buttonDown.Enable(False)
        panelControlSizer.Add(self.buttonDown, 2, wx.ALIGN_CENTER_HORIZONTAL | wx.BOTTOM, 5)
        panelControlSizer.AddStretchSpacer(1)
        self.buttonDown.Bind(wx.EVT_KEY_DOWN, self.OnKeyDown)

        # Level Status Panel
        self.panelLevelStatus = wx.Panel(self.panelBottom)
        panelLevelStatusSizer = wx.BoxSizer(wx.VERTICAL)
        self.panelLevelStatus.SetSizer(panelLevelStatusSizer)
        panelBottomSizer.Add(self.panelLevelStatus, 1, wx.EXPAND)
        self.panelLevelStatus.Bind(wx.EVT_KEY_DOWN, self.OnKeyDown)

        # Level Status Gauge
        self.gaugeLevel = wx.Gauge(parent = self.panelLevelStatus, style = wx.GA_VERTICAL)
        self.gaugeLevel.SetRange(1)
        self.gaugeLevel.SetValue(1)
        panelLevelStatusSizer.Add(self.gaugeLevel, 1, wx.ALIGN_CENTER_HORIZONTAL | wx.BOTTOM | wx.LEFT, 10)
        self.gaugeLevel.Bind(wx.EVT_KEY_DOWN, self.OnKeyDown)

        # Level Panel
        self.panelLevel = wx.Panel(self.panelBottom)
        panelLevelSizer = wx.BoxSizer(wx.VERTICAL)
        self.panelLevel.SetSizer(panelLevelSizer)
        panelBottomSizer.Add(self.panelLevel, 2, wx.EXPAND)
        self.panelLevel.Bind(wx.EVT_KEY_DOWN, self.OnKeyDown)

        # Start Button
        self.buttonStart = wx.Button(parent = self.panelLevel, label = "Start")
        self.buttonStart.Bind(wx.EVT_BUTTON, self.OnStart)
        self.buttonStart.Enable(False)
        panelLevelSizer.Add(self.buttonStart, 0, wx.ALIGN_RIGHT | wx.RIGHT | wx.TOP, 5)
        self.buttonStart.Bind(wx.EVT_KEY_DOWN, self.OnKeyDown)

        # Next Button
        self.buttonNext = wx.Button(parent = self.panelLevel, label = "Next")
        self.buttonNext.Bind(wx.EVT_BUTTON, self.OnNext)
        self.buttonNext.Enable(False)
        panelLevelSizer.Add(self.buttonNext, 0, wx.ALIGN_RIGHT | wx.RIGHT | wx.TOP, 5)
        self.buttonNext.Bind(wx.EVT_KEY_DOWN, self.OnKeyDown)

        self.buttonSave = wx.Button(parent = self.panelLevel, label = "Save")
        self.buttonSave.Bind(wx.EVT_BUTTON, self.OnSave)
        self.buttonSave.Enable(False)
        panelLevelSizer.Add(self.buttonSave, 0, wx.ALIGN_RIGHT | wx.RIGHT | wx.TOP | wx.BOTTOM, 5)
        self.buttonSave.Bind(wx.EVT_KEY_DOWN, self.OnKeyDown)

        self.SetSizerAndFit(outerPanelSizer)
        #self.SetSizer(outerPanelSizer)

    def EnumerateSerialPorts(self):
        # List all available serial ports
        serialPortList = serial.tools.list_ports.comports()

        # Fill Combobox, remember selected value
        selection = self.comboBoxSerial.GetValue()
        self.comboBoxSerial.Clear()
        for port in serialPortList:
            self.comboBoxSerial.Append(port[0])
            if port[0] == selection:
                self.comboBoxSerial.SetStringSelection(selection)

        # Select first port if nothing is selected
        if not selection:
            self.comboBoxSerial.SetStringSelection(serialPortList[0][0])

        self.panelSerial.Layout()

        # Check every 5 seconds for new available serial ports
        wx.FutureCall(POLL_SERIAL_PORTS_INTERVALL, self.EnumerateSerialPorts)

    def EnableDisableUI(self):
        if not self.isConnected:
            self.buttonSerial.SetLabel("Connect")
            self.labelX.SetLabel("X: ??.??")
            self.labelY.SetLabel("Y: ??.??")
            self.labelZ.SetLabel("Z: ??.??")
            self.labelZc.SetLabel("Zc: ??.???")
        else:
            self.buttonSerial.SetLabel("Disconnect")

        self.panelStatus.Layout()
        self.panelSerial.Layout()

        self.comboBoxSerial.Enable(not self.isConnected)
        self.comboBoxBaud.Enable(not self.isConnected)

        self.buttonHome.Enable(self.isConnected)
        self.buttonOff.Enable(self.isConnected)
        self.buttonUp.Enable(self.isConnected)
        self.buttonDown.Enable(self.isConnected)
        self.stepSize.Enable(self.isConnected)
        self.bedTemperature.Enable(self.isConnected)

        self.buttonStart.Enable(False)
        self.buttonNext.Enable(False)
        self.buttonSave.Enable(False)
        if self.isConnected:
            if not self.isLeveling:
                self.buttonSave.Enable(True)
                if self.meshPoints > 0:
                    self.buttonStart.Enable(True)

            if self.isLeveling:
                self.buttonNext.Enable(True)

    def OnSave(self, event):
        if self.isLeveling:
            print "Leveling is already in progress!"
            return

        self.protocol.write_line(GCODE_EEPROM_SAVE)

    def OnStart(self, event):
        if self.isLeveling:
            print "Leveling is already in progress!"
            return

        self.isLeveling = True
        self.currentPoint = 0
        self.gauge.SetValue(0)
        self.protocol.write_line(GCODE_MESH_START)
        self.gaugeLevel.SetValue(0)
        self.EnableDisableUI()

    def OnNext(self, event):
        if not self.isLeveling:
            print "Start leveling before proceeding!"
            return

        self.gauge.SetValue(0)
        self.protocol.write_line(GCODE_MESH_NEXT)
        self.currentPoint += 1
        if self.currentPoint >= self.meshPoints:
            self.isLeveling = False
        self.gaugeLevel.SetValue(self.currentPoint)
        self.EnableDisableUI()
        self.currentZ = self.startZ
        self.labelZc.SetLabel("Zc: {}".format(self.currentZ))
        self.panelStatus.Layout()

    def OnNewStepSize(self, event):
        print "New Step Size: {}".format(self.stepSize.GetValue())
        self.step = float(self.stepSize.GetValue())

    def OnKeyDown(self, event):
        key = event.GetKeyCode()

        if self.isConnected:
            if key == wx.WXK_DOWN:
                self.OnDown(event)
            elif key == wx.WXK_UP:
                self.OnUp(event)
            elif key == wx.WXK_RIGHT:
                self.OnNext(event)

    def OnUp(self, event):
        self.currentZ += self.step
        print "Moving up to {}".format(self.currentZ)
        self.gauge.SetValue(0)
        self.protocol.write_line(GCODE_MOVE_Z.format(self.currentZ))
        self.labelZc.SetLabel("Zc: {}".format(self.currentZ))
        self.panelStatus.Layout()

    def OnDown(self, event):
        if self.currentZ <= 0.0:
            print "Can't move further down!"
            return

        self.currentZ -= self.step
        print "Moving down to {}".format(self.currentZ)
        self.gauge.SetValue(0)
        self.protocol.write_line(GCODE_MOVE_Z.format(self.currentZ))
        self.labelZc.SetLabel("Zc: {}".format(self.currentZ))
        self.panelStatus.Layout()

    def OnStepperOff(self, event):
        self.gauge.SetValue(0)
        self.protocol.write_line(GCODE_STOP_IDLE_HOLD)

    def OnHome(self, event):
        self.gauge.SetValue(0)
        self.protocol.write_line(GCODE_MOVE_TO_ORIGIN)

    def OnStatus(self, event):
        if event.GetValue() == 0:
            self.OnDone(event)
        else:
            self.OnBusy(event)

    def OnBusy(self, event):
        val = self.gauge.GetValue()
        if val < self.gauge.GetRange():
            self.gauge.SetValue(val + 1)

    def OnDone(self, event):
        self.gauge.SetValue(self.gauge.GetRange())

    def PollMeshData(self):
        if self.isConnected:
            self.protocol.write_line(GCODE_MESH_INFO)

    def PollMeshDataRetry(self):
        if self.meshPoints <= 0:
            self.PollMeshData()
            wx.FutureCall(MESH_DATA_RETRY_TIMEOUT, self.PollMeshDataRetry)

    def OnMeshPoints(self, event):
        x = int(event.GetX())
        y = int(event.GetY())
        self.meshPoints = x * y
        print "Mesh Points: X: {} Y: {} --> {}".format(x, y, self.meshPoints)
        self.gaugeLevel.SetRange(self.meshPoints)
        self.gaugeLevel.SetValue(0)
        self.EnableDisableUI()

    def OnMeshZ(self, event):
        self.startZ = float(event.GetZ())
        self.currentZ = float(event.GetZ())
        self.labelZc.SetLabel("Zc: {}".format(self.currentZ))
        self.gauge.SetValue(BUSY_PROCESSING_STEPS)
        print "Mesh starting height: {}".format(self.currentZ)

    def PollPosition(self):
        if self.isConnected:
            #if not self.isLeveling:
            self.protocol.write_line(GCODE_GET_CURRENT_POSITION)
            wx.FutureCall(POLL_COORDINATES_INTERVALL, self.PollPosition)

    def OnCoordinates(self, event):
        data = event.GetValue()
        print "New coordinates {}".format(data)
        afterX = data.split("X: ")[1]
        x = afterX.split(" ")[0]
        afterY = data.split("Y: ")[1]
        y = afterY.split(" ")[0]
        afterZ = data.split("Z: ")[1]
        z = afterZ.split(" ")[0]

        self.labelX.SetLabel("X: {}".format(x))
        self.labelY.SetLabel("Y: {}".format(y))
        self.labelZ.SetLabel("Z: {}".format(z))

        self.panelStatus.Layout()

    def OnTemperature(self, event):
        print "Current Bed Temperature: {}".format(event.GetBed())
        self.bedTemperatureLabel.SetLabel("Bed: {}".format(event.GetBed()))

    def OnNewBedTemperature(self, event):
        self.protocol.write_line(GCODE_SET_BED_TEMP.format(float(self.bedTemperature.GetValue())))

    def PollTemperature(self):
        if self.isConnected:
            self.protocol.write_line(GCODE_GET_TEMP)
            wx.FutureCall(POLL_TEMPERATURE_INTERVALL, self.PollTemperature)

    def OnConnectDisconnect(self, Event):
        self.isLeveling = False
        self.currentPoint = 0
        if self.isConnected:
            self.thread.close()
            self.serial.close()
            self.isConnected = False
            self.gauge.SetValue(0)
            self.gaugeLevel.SetRange(1)
            self.gaugeLevel.SetValue(1)
            self.EnableDisableUI()

            # Recreate thread, so we can start it again next time
            self.thread = serial.threaded.ReaderThread(self.serial, GCodeReader)
        else:
            self.serial.baudrate = self.comboBoxBaud.GetValue()
            self.serial.port = self.comboBoxSerial.GetValue()
            try:
                self.serial.open()
            except serial.SerialException as msg:
                wx.MessageBox("SerialException: {}".format(msg), "Error", wx.OK | wx.ICON_ERROR)
            except OSError as msg:
                wx.MessageBox("OSError: {}".format(msg), "Error", wx.OK | wx.ICON_ERROR)
            if self.serial.isOpen():
                self.isConnected = True
                self.thread.start()
                self.transport, self.protocol = self.thread.connect()
                self.protocol.setParent(self)
                self.meshPoints = -1
                self.EnableDisableUI()
                self.PollPosition()
                self.PollMeshData()
                self.PollTemperature()
                wx.FutureCall(MESH_DATA_RETRY_TIMEOUT, self.PollMeshDataRetry)

    def OnQuit(self, Event):
        if not self.hasQuit:
            self.hasQuit = True
            if self.isConnected:
                print("Closing serial port...")
                self.thread.close()
                self.serial.close()
                self.isConnected = False
        self.Destroy()

# App wrapper class
class WizardApp(wx.App):
    def __init__(self, *args, **kwargs):
        wx.App.__init__(self, *args, **kwargs)
        self.Bind(wx.EVT_ACTIVATE_APP, self.OnActivate)

    def OnInit(self):
        frame = WizardFrame(parent = None, title = "Bed Leveling Wizard", style = wx.SYSTEM_MENU | wx.CAPTION | wx.CLOSE_BOX | wx.CLIP_CHILDREN)
        frame.Centre()
        frame.Show()
        return True

    def BringWindowToFront(self):
        self.GetTopWindow().Raise()

    def OnActivate(self, event):
        if event.GetActive():
            self.BringWindowToFront()
        event.Skip()

    def MacReopenApp(self):
        self.BringWindowToFront()

def main():
    app = WizardApp(False)
    app.MainLoop()

if __name__ == '__main__':
    main()

