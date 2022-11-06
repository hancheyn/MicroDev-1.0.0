# #################################################
# MicroDev Project: View
# Date: May 2, 2022
# Description: View Class
# Authors:
# Connor Inglet
# Dylan Vetter
# Corey Noura
# Nathan Hanchey
# #################################################

# import time
from tkinter import *
import RPi.GPIO as GPIO
import Bigfoot as bigfoot

# Creates main GUI window and its subframe for data to be displayed
root = Tk()
root.wm_title("MicroDev Tester")
root.attributes('-fullscreen', True)
root.config(cursor="none")
root.configure(background="black")
frame = Frame(root)
frame.place(relx=0.0, rely=0.0, relheight=1.0, relwidth=1.0)
frame.configure(background="black")

global progress
global progressBar

# ----------------------------------------------------------------------
# Description: Standby Screen while waiting for subject board connection
# Parameters: None
# Returns: None
# ----------------------------------------------------------------------
def setStandbyScreen():
    # Clears main window's subframe
    for widget in frame.winfo_children():
        if widget.winfo_exists():
            widget.destroy()
    frame.place(relx=0.0, rely=0.0, relheight=1.0, relwidth=1.0)

    subframe = Frame(frame)
    subframe.configure(background="black", highlightbackground="white", highlightcolor="white", highlightthickness=5)
    report = Label(subframe, text="Place dev board onto header\ninterface and connect cable", background='black', font=("futura", 35), fg="green")

    leftButton = Label(frame, text="", background='black', font=("futura", 35), fg="green", highlightthickness=5)
    leftButton.config(highlightbackground="white", highlightcolor="white")
    middleButton = Label(frame, text="Shutdown", background='black', font=("futura", 35), fg="green", highlightthickness=5)
    middleButton.config(highlightbackground="white", highlightcolor="white")
    rightButton = Label(frame, text="", background='black', font=("futura", 35), fg="green", highlightthickness=5)
    rightButton.config(highlightbackground="white", highlightcolor="white")

    subframe.place(relx=0.0, rely=0.0, relwidth=1.0, relheight=0.75)
    report.place(relx=0.5, rely=0.5, relwidth=1.0, relheight=1.0, anchor="center")

    leftButton.place(relx=0.025, rely=0.775, relheight=0.2, relwidth=0.3)
    middleButton.place(relx=0.35, rely=0.775, relheight=0.2, relwidth=0.3)
    rightButton.place(relx=0.675, rely=0.775, relheight=0.2, relwidth=0.3)

    # Updates main window
    root.update()

# ----------------------------------------------------------------------
# Description: Start Screen waits for start button to be pressed
# Parameters: [string] subject board type
# Returns: None
# ----------------------------------------------------------------------
def setStartScreen(board_type):
    # Clears main window's subframe
    for widget in frame.winfo_children():
        if widget.winfo_exists():
            widget.destroy()
    frame.place(relx=0.0, rely=0.0, relheight=1.0, relwidth=1.0)

    subframe = Frame(frame)
    subframe.configure(background="black", highlightbackground="white", highlightcolor="white", highlightthickness=5)
    report = Label(subframe, text=board_type, background='black', font=("futura", 35), fg="green")

    leftButton = Label(frame, text="", background='black', font=("futura", 35), fg="green", highlightthickness=5)
    leftButton.config(highlightbackground="white", highlightcolor="white")
    middleButton = Label(frame, text="Start Test", background='black', font=("futura", 35), fg="green", highlightthickness=5)
    middleButton.config(highlightbackground="white", highlightcolor="white")
    rightButton = Label(frame, text="", background='black', font=("futura", 35), fg="green", highlightthickness=5)
    rightButton.config(highlightbackground="white", highlightcolor="white")

    subframe.place(relx=0.0, rely=0.0, relwidth=1.0, relheight=0.75)
    report.place(relx=0.5, rely=0.5, relwidth=1.0, relheight=1.0, anchor="center")

    leftButton.place(relx=0.025, rely=0.775, relheight=0.2, relwidth=0.3)
    middleButton.place(relx=0.35, rely=0.775, relheight=0.2, relwidth=0.3)
    rightButton.place(relx=0.675, rely=0.775, relheight=0.2, relwidth=0.3)

    # Updates main window
    root.update()

# ----------------------------------------------------------------------
# Description: Displays a message to GUI
# Parameters: [string] any string message to display
# Returns: None
# ----------------------------------------------------------------------
def setMessageScreen(message):
    # Clears main window's subframe
    for widget in frame.winfo_children():
        if widget.winfo_exists():
            widget.destroy()
    frame.place(relx=0.0, rely=0.0, relheight=1.0, relwidth=1.0)

    subframe = Frame(frame)
    subframe.configure(background="black", highlightbackground="white", highlightcolor="white", highlightthickness=5)
    report = Label(subframe, text=message, background='black', font=("futura", 35), fg="green")

    leftButton = Label(frame, text="", background='black', font=("futura", 35), fg="green", highlightthickness=5)
    leftButton.config(highlightbackground="white", highlightcolor="white")
    middleButton = Label(frame, text="", background='black', font=("futura", 35), fg="green",
                         highlightthickness=5)
    middleButton.config(highlightbackground="white", highlightcolor="white")
    rightButton = Label(frame, text="", background='black', font=("futura", 35), fg="green", highlightthickness=5)
    rightButton.config(highlightbackground="white", highlightcolor="white")

    subframe.place(relx=0.0, rely=0.0, relwidth=1.0, relheight=0.75)
    report.place(relx=0.5, rely=0.5, relwidth=1.0, relheight=1.0, anchor="center")

    leftButton.place(relx=0.025, rely=0.775, relheight=0.2, relwidth=0.3)
    middleButton.place(relx=0.35, rely=0.775, relheight=0.2, relwidth=0.3)
    rightButton.place(relx=0.675, rely=0.775, relheight=0.2, relwidth=0.3)

    # Updates main window
    root.update()


# ----------------------------------------------------------------------
# Description: Flash Screen indicates flashing of microcontroller is in process
# Parameters: None
# Returns: None
# ----------------------------------------------------------------------
def setFlashScreen():
    # Clears main window's subframe
    for widget in frame.winfo_children():
        if widget.winfo_exists():
            widget.destroy()
    frame.place(relx=0.0, rely=0.0, relheight=1.0, relwidth=1.0)

    subframe = Frame(frame)
    subframe.configure(background="black", highlightbackground="white", highlightcolor="white", highlightthickness=5)
    report = Label(subframe, text="Flashing Software to\nSubject Board...", background='black', font=("futura", 35), fg="green")

    leftButton = Label(frame, text="", background='black', font=("futura", 35), fg="green", highlightthickness=5)
    leftButton.config(highlightbackground="white", highlightcolor="white")
    middleButton = Label(frame, text="", background='black', font=("futura", 35), fg="green", highlightthickness=5)
    middleButton.config(highlightbackground="white", highlightcolor="white")
    rightButton = Label(frame, text="", background='black', font=("futura", 35), fg="green", highlightthickness=5)
    rightButton.config(highlightbackground="white", highlightcolor="white")

    subframe.place(relx=0.0, rely=0.0, relwidth=1.0, relheight=0.75)
    report.place(relx=0.5, rely=0.5, relwidth=1.0, relheight=1.0, anchor="center")

    leftButton.place(relx=0.025, rely=0.775, relheight=0.2, relwidth=0.3)
    middleButton.place(relx=0.35, rely=0.775, relheight=0.2, relwidth=0.3)
    rightButton.place(relx=0.675, rely=0.775, relheight=0.2, relwidth=0.3)

    # Updates main window
    root.update()


# ----------------------------------------------------------------------
# Description: Process bar indicating number of tests completed
# Parameters: [int] percent of tests completed
# Returns: None
# ----------------------------------------------------------------------
def setRunningScreen(percent):
    global progress
    global progressBar

    try:
        progress.configure(text=str(percent) + "%")
        if 0 < percent <= 100:
            progressBar.place(relx=0.0, rely=0.0, relwidth=float(percent)/100, relheight=1.0)
        else:
            progressBar.place(relx=0.0, rely=0.0, relwidth=0, relheight=1.0)

        progress.update()
        progressBar.update()
    except Exception:
        for widget in frame.winfo_children():
            if widget.winfo_exists():
                widget.destroy()
        frame.place(relx=0.0, rely=0.0, relheight=1.0, relwidth=1.0)

        subframe = Frame(frame)
        subframe.configure(background="black", highlightbackground="white", highlightcolor="white", highlightthickness=5)
        report = Label(subframe, text="Running Tests...", background='black', font=("futura", 35), fg="green")
        progress = Label(subframe, text=str(percent) + "%", background='black', font=("futura", 35), fg="green")
        progressBarFrame = Frame(subframe)
        progressBarFrame.configure(background="black", highlightbackground="white", highlightcolor="white", highlightthickness=5)
        progressBar = Label(progressBarFrame, text="", background="green", fg="black")

        leftButton = Label(frame, text="", background='black', font=("futura", 35), fg="green", highlightthickness=5)
        leftButton.config(highlightbackground="white", highlightcolor="white")
        middleButton = Label(frame, text="", background='black', font=("futura", 35), fg="green", highlightthickness=5)
        middleButton.config(highlightbackground="white", highlightcolor="white")
        rightButton = Label(frame, text="", background='black', font=("futura", 35), fg="green", highlightthickness=5)
        rightButton.config(highlightbackground="white", highlightcolor="white")

        subframe.place(relx=0.0, rely=0.0, relwidth=1.0, relheight=0.75)
        report.place(relx=0.5, rely=0.2, relwidth=1.0, relheight=0.3, anchor="center")
        progress.place(relx=0.5, rely=0.5, relwidth=1.0, relheight=0.3, anchor="center")
        progressBarFrame.place(relx=0.5, rely=0.8, relwidth=0.80, relheight=0.3, anchor="center")
        if 0 < percent <= 100:
            progressBar.place(relx=0.0, rely=0.0, relwidth=float(percent)/100, relheight=1.0)
        else:
            progressBar.place(relx=0.0, rely=0.0, relwidth=0, relheight=1.0)

        leftButton.place(relx=0.025, rely=0.775, relheight=0.2, relwidth=0.3)
        middleButton.place(relx=0.35, rely=0.775, relheight=0.2, relwidth=0.3)
        rightButton.place(relx=0.675, rely=0.775, relheight=0.2, relwidth=0.3)

        # Updates main window
        root.update()


# ----------------------------------------------------------------------
# Description: Shows the results of each test to the GUI
# Parameters: [string array] array of the general results
# Returns: None
# ----------------------------------------------------------------------
def setResultsScreen(pass_array):
    # Clears main window's subframe
    for widget in frame.winfo_children():
        if widget.winfo_exists():
            widget.destroy()
    frame.place(relx=0.0, rely=0.0, relheight=1.0, relwidth=1.0)

    # Creates scrolling subframe and button icons
    subframe = Frame(frame)
    subframe.configure(background="black", highlightbackground="white", highlightcolor="white", highlightthickness=5)
    leftButton = Label(frame, text="Save", background='black', font=("futura", 35), fg="green", highlightthickness=5)
    leftButton.config(highlightbackground="white", highlightcolor="white")
    middleButton = Label(frame, text="Details", background='black', font=("futura", 35), fg="green", highlightthickness=5)
    middleButton.config(highlightbackground="white", highlightcolor="white")
    rightButton = Label(frame, text="New Test", background='black', font=("futura", 35), fg="green", highlightthickness=5)
    rightButton.config(highlightbackground="white", highlightcolor="white")

    subframe.place(relx=0.0, rely=0.0, relwidth=1.0, relheight=0.75)
    leftButton.place(relx=0.025, rely=0.775, relheight=0.2, relwidth=0.3)
    middleButton.place(relx=0.35, rely=0.775, relheight=0.2, relwidth=0.3)
    rightButton.place(relx=0.675, rely=0.775, relheight=0.2, relwidth=0.3)

     # Displays first page worth of entries
    report = Text(subframe, width=100, height=100, wrap=WORD, background='black', font=("futura", 35), fg="green", padx=10, pady=10)
    for i in range(len(pass_array)):
        report.insert(END, str(pass_array[i]) + "\n")
    report.place(relx=0.0, rely=0.0, relwidth=1.0, relheight=1.0)

    # Updates main window
    root.update()


# ----------------------------------------------------------------------
# Description: Shows the detailed test results on the GUI
# Parameters: [string array] detailed test results
# Returns: None
# ----------------------------------------------------------------------
def setDetailTestScreen(detailed_report):
    # Clears main window's subframe
    for widget in frame.winfo_children():
        if widget.winfo_exists():
            widget.destroy()
    frame.place(relx=0.0, rely=0.0, relheight=1.0, relwidth=1.0)

    # Creates scrolling subframe and button icons
    subframe = Frame(frame)
    subframe.configure(background="black", highlightbackground="white", highlightcolor="white", highlightthickness=5)
    leftButton = Label(frame, text="Page Down", background='black', font=("futura", 35), fg="green", highlightthickness=5)
    leftButton.config(highlightbackground="white", highlightcolor="white")
    middleButton = Label(frame, text="Next", background='black', font=("futura", 35), fg="green", highlightthickness=5)
    middleButton.config(highlightbackground="white", highlightcolor="white")
    rightButton = Label(frame, text="Page Up", background='black', font=("futura", 35), fg="green", highlightthickness=5)
    rightButton.config(highlightbackground="white", highlightcolor="white")

    subframe.place(relx=0.0, rely=0.0, relwidth=1.0, relheight=0.75)
    leftButton.place(relx=0.025, rely=0.775, relheight=0.2, relwidth=0.3)
    middleButton.place(relx=0.35, rely=0.775, relheight=0.2, relwidth=0.3)
    rightButton.place(relx=0.675, rely=0.775, relheight=0.2, relwidth=0.3)

     # Initializes variables used for indexing through array for scrolling
    button_press = None
    current_page = 0
    page_size = 5
    num_pages = int(len(detailed_report) / page_size)
    if((len(detailed_report) % page_size) == 0):
        num_pages -= 1

     # Displays first page worth of entries
    report = Text(subframe, width=100, height=100, wrap=WORD, background='black', font=("futura", 35), fg="green", padx=10, pady=10)
    for i in range(len(detailed_report)):
        if i < current_page * page_size:
            continue
        elif i > ((current_page * page_size) + page_size) - 1:
            continue
        else:
            report.insert(END, str(detailed_report[i]) + "\n")
    report.place(relx=0.0, rely=0.0, relwidth=1.0, relheight=1.0)

    # Updates main window
    root.update()

     # Continues to allow for scrolling by polling buttons until center button is pressed
    while button_press != "center":

        # Blocking call that waits for next button press and stores it
        button_press = bigfoot.pollButtons()

         # Increments variables used to index for scrolling
        if button_press == "right" and current_page < num_pages:
            current_page += 1
        elif button_press == "left" and current_page > 0:
            current_page -= 1
        else:
            continue

         # Wipes out previous entires from subframe
        for widget in subframe.winfo_children():
            if widget.winfo_exists():
                widget.destroy()
        subframe.place(relx=0.0, rely=0.0, relwidth=1.0, relheight=0.75)

         # Creates new entries to go in subframe that acts like scrolling to next page
        report = Text(subframe, width=100, height=100, wrap=WORD, font=("futura", 35), background='black', fg="green", padx=10, pady=10)
        for i in range(len(detailed_report)):
            if i < current_page * page_size:
                continue
            elif i > ((current_page * page_size) + page_size) - 1:
                continue
            else:
                report.insert(END, detailed_report[i] + "\n")
        report.place(relx=0.0, rely=0.0, relwidth=1.0, relheight=1.0)

         # Updates main window
        root.update()


# ----------------------------------------------------------------------
# Description: Save screen indicates which tests were saved
# Parameters: [string] "Success" indicates a successful upload
# Returns: None
# ----------------------------------------------------------------------
def setSaveScreen(save_condition):
    # Clears main window's subframe
    for widget in frame.winfo_children():
        if widget.winfo_exists():
            widget.destroy()
    frame.place(relx=0.0, rely=0.0, relheight=1.0, relwidth=1.0)

    subframe = Frame(frame)
    if save_condition == "Success":
        subframe.configure(background="black", highlightbackground="white", highlightcolor="white", highlightthickness=5)
        report = Label(subframe, text="Saved Test Results to USB", background='black', font=("futura", 35), fg="green")
    else:
        subframe.configure(background="black", highlightbackground="white", highlightcolor="white", highlightthickness=5)
        report = Label(subframe, text="Results Failed to Save on USB", background='black', font=("futura", 35), fg="red")

    leftButton = Label(frame, text="", background='black', font=("futura", 35), fg="green", highlightthickness=5)
    leftButton.config(highlightbackground="white", highlightcolor="white")
    middleButton = Label(frame, text="", background='black', font=("futura", 35), fg="green", highlightthickness=5)
    middleButton.config(highlightbackground="white", highlightcolor="white")
    rightButton = Label(frame, text="Back", background='black', font=("futura", 35), fg="green", highlightthickness=5)
    rightButton.config(highlightbackground="white", highlightcolor="white")

    subframe.place(relx=0.0, rely=0.0, relwidth=1.0, relheight=0.75)
    report.place(relx=0.5, rely=0.5, relwidth=1.0, relheight=1.0, anchor="center")

    leftButton.place(relx=0.025, rely=0.775, relheight=0.2, relwidth=0.3)
    middleButton.place(relx=0.35, rely=0.775, relheight=0.2, relwidth=0.3)
    rightButton.place(relx=0.675, rely=0.775, relheight=0.2, relwidth=0.3)

    # Updates main window
    root.update()


# ----------------------------------------------------------------------
# Description: Shutdown Screen indicates that the Device is shutting down
# Parameters: None
# Returns: None
# ----------------------------------------------------------------------
def setShutdownScreen():
    # Clears main window's subframe
    for widget in frame.winfo_children():
        if widget.winfo_exists():
            widget.destroy()
    frame.place(relx=0.0, rely=0.0, relheight=1.0, relwidth=1.0)

    subframe = Frame(frame)
    subframe.configure(background="black", highlightbackground="white", highlightcolor="white", highlightthickness=5)
    report = Label(subframe, text="Shutdown in Progress...", background='black', font=("futura", 35), fg="green")

    leftButton = Label(frame, text="", background='black', font=("futura", 35), fg="green", highlightthickness=5)
    leftButton.config(highlightbackground="white", highlightcolor="white")
    middleButton = Label(frame, text="", background='black', font=("futura", 35), fg="green", highlightthickness=5)
    middleButton.config(highlightbackground="white", highlightcolor="white")
    rightButton = Label(frame, text="", background='black', font=("futura", 35), fg="green", highlightthickness=5)
    rightButton.config(highlightbackground="white", highlightcolor="white")

    subframe.place(relx=0.0, rely=0.0, relwidth=1.0, relheight=0.75)
    report.place(relx=0.5, rely=0.5, relwidth=1.0, relheight=1.0, anchor="center")

    leftButton.place(relx=0.025, rely=0.775, relheight=0.2, relwidth=0.3)
    middleButton.place(relx=0.35, rely=0.775, relheight=0.2, relwidth=0.3)
    rightButton.place(relx=0.675, rely=0.775, relheight=0.2, relwidth=0.3)

    # Updates main window
    root.update()


# ----------------------------------------------------------------------
# Description: Removal Screen indicates that it is time to remove the subject board.
# Parameters: None
# Returns: None
# ----------------------------------------------------------------------
def setRemovalScreen():
    # Clears main window's subframe
    for widget in frame.winfo_children():
        if widget.winfo_exists():
            widget.destroy()
    frame.place(relx=0.0, rely=0.0, relheight=1.0, relwidth=1.0)
    
    subframe = Frame(frame)
    subframe.configure(background="black", highlightbackground="white", highlightcolor="white", highlightthickness=5)
    report = Label(subframe, text="Remove Dev Board From Fixture", background='black', font=("futura", 35), fg="green")

    leftButton = Label(frame, text="", background='black', font=("futura", 35), fg="green", highlightthickness=5)
    leftButton.config(highlightbackground="white", highlightcolor="white")
    middleButton = Label(frame, text="", background='black', font=("futura", 35), fg="green", highlightthickness=5)
    middleButton.config(highlightbackground="white", highlightcolor="white")
    rightButton = Label(frame, text="", background='black', font=("futura", 35), fg="green", highlightthickness=5)
    rightButton.config(highlightbackground="white", highlightcolor="white")
    
    subframe.place(relx=0.0, rely=0.0, relwidth=1.0, relheight=0.75)
    report.place(relx=0.5, rely=0.5, relwidth=1.0, relheight=1.0, anchor="center")

    leftButton.place(relx=0.025, rely=0.775, relheight=0.2, relwidth=0.3)
    middleButton.place(relx=0.35, rely=0.775, relheight=0.2, relwidth=0.3)
    rightButton.place(relx=0.675, rely=0.775, relheight=0.2, relwidth=0.3)

    # Updates main window
    root.update() 
    

# ----------------------------------------------------------------------
# Description:
# Parameters: None
# Returns: None
# ----------------------------------------------------------------------
def pollButtons():
    
    # Initializes GPIO button for polling
    BTN_L = 14
    BTN_C = 15
    BTN_R = 18
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup([BTN_L, BTN_C, BTN_R], GPIO.IN, pull_up_down=GPIO.PUD_UP)
    
    # Waits until user presses button
    while GPIO.input(BTN_L) and GPIO.input(BTN_C) and GPIO.input(BTN_R):
        continue
        
    # Returns string for which button was pressed and waits until the user lets go
    if not GPIO.input(BTN_L):
        while not GPIO.input(BTN_L):
            continue
        return "left"
    elif not GPIO.input(BTN_C):
        while not GPIO.input(BTN_C):
            continue
        return "center"
    elif not GPIO.input(BTN_R):
        while not GPIO.input(BTN_R):
            continue
        return "right"
    else:    
        return None