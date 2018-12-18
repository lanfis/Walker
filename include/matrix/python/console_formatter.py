#!/usr/bin/env python
# license removed for brevity
import sys

class Console_Formatter:
    
    def INFO(self, msg, node_name=None):
        MSG = None if node_name is "" else self.GREEN+"[{}]".format(self.node_name)
        MSG = MSG + self.NO_COLOR + msg +self.NO_COLOR
        return MSG
        
    def DEBUG(self, msg, node_name=None):
        MSG = None if node_name is "" else self.BLUE+"[{}]".format(self.node_name)
        MSG = MSG + self.NO_COLOR + msg +self.NO_COLOR
        return MSG
        
    def WARN(self, msg, node_name=None):
        MSG = None if node_name is "" else self.YELLOW+"[{}]".format(self.node_name)
        MSG = MSG + self.YELLOW + msg   +self.NO_COLOR
        return MSG
        
    def ERR(self, msg, node_name=None):
        MSG = None if node_name is "" else self.RED+"[{}]".format(self.node_name)
        MSG = MSG + self.RED + msg  +self.NO_COLOR
        return MSG
        
    def FATAL(self, msg, node_name=None):
        MSG = None if node_name is "" else self.RED+"[{}]".format(self.node_name)
        MSG = MSG + self.RED + msg  +self.NO_COLOR
        return MSG
        
    def __init__(self, node_name=None):
        self.node_name = node_name
        self.NO_COLOR   = "\033[0m"
        self.BLACK      = "\033[30m"
        self.RED        = "\033[31m"
        self.GREEN      = "\033[32m"
        self.YELLOW     = "\033[33m"
        self.BLUE       = "\033[34m"
        self.MAGENTA    = "\033[35m"
        self.CYAN       = "\033[36m"
        self.LIGHTGRAY  = "\033[37m"
