#Remember to
  #pip install python-rtmidi

#https://pypi.org/project/python-rtmidi/
#https://github.com/SpotlightKid/python-rtmidi
#https://www.cs.cmu.edu/~music/cmsip/readings/GMSpecs_Patches.html

#MIDI Instrument
#https://fmslogo.sourceforge.io/manual/midi-instrument.html

#twinkle!
#60 60 67 67 69 69 67,1.7 65 65 64 64 62 62 60,1.7 67 67 65 65 64 64 62,1.7 67 67 65 65 64 64 62,1.7 60 60 67 67 69 69 67,1.7 65 65 64 64 62 62 60
import time
import rtmidi

def play_midi(notes, instrument):
    # Initialize MIDI output
    midi_out = rtmidi.MidiOut()
    available_ports = midi_out.get_ports()

    if available_ports:
        midi_out.open_port(0)
    else:
        midi_out.open_virtual_port("My virtual output")

    # Instrument
    program_change = [0xC0, instrument]
    midi_out.send_message(program_change)

    for note_info in notes:
        if ',' in note_info:
            note_number, duration = note_info.split(',')
            duration = float(duration)
        else:
            note_number = note_info
            # Default duration 1 sec
            duration = 1.0  

        note_number = int(note_number)

        # Pause
        if note_number == -1:  
            time.sleep(duration)
            continue

        note_on = [0x90, note_number, 127]
        midi_out.send_message(note_on)

        time.sleep(duration)

        # Send MIDI note off message
        note_off = [0x80, note_number, 0]
        midi_out.send_message(note_off)

    # Close MIDI output
    del midi_out

if __name__ == "__main__":
    # Input notes, durations, and pauses separated by spaces
    # e.g., "60,1.5 62,1 64,0.5 -1,1" where -1 indicates a pause of 1 sec
    notes_input = input("Enter MIDI notes, durations, and pauses(-1,duration) separated by spaces: ")
    notes = notes_input.split()

    # Select instrument
    instrument = int(input("Enter instrument number (0-127): "))

    play_midi(notes, instrument)
