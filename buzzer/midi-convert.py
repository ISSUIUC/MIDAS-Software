from mido import MidiFile

FILE_NAME = "heyjude.mid"
# you can set this to be whatever midi file you want

mid = MidiFile(f"midi-files/{FILE_NAME}") 
# for i in range(1, 5):
track_num = 1

track = mid.tracks[track_num]
print(track)
notes_and_times = []

note_offset = 0
smallest_time = 1

time_tracker = 0
for msg in track:
    time_tracker += msg.time
    if msg.type == "note_on":
        notes_and_times.append((msg.note - note_offset, time_tracker / smallest_time))


notes = [thing[0] for thing in notes_and_times]
times = [(thing[1]) for thing in notes_and_times]

# for i, note, time in enumerate(notes_and_times):

times_2 = [int(time - times[i]) for i, time in enumerate(times[1:])] + [500]
notes_2 = [(440*2**((note-69)/12)) for note in notes]
# print(times_2)
# print(min(times_2))

notes_str = "int[] notes = {" + ", ".join([str(note) for note in notes_2]) + "};"
times_2 = [round((time)) for time in times_2]

new_notes = []
new_times = []

for i, (note, duration) in enumerate(list(zip(notes_2, times_2))):
    while duration > 255:
        new_times.append(255)
        duration -= 255
        new_notes.append(note)
    new_times.append(duration)
    new_notes.append(note)

    

notes_final = "{"

for note, duration in zip(new_notes, new_times):
    notes_final += "Sound{" + str(round(note)) + ", static_cast<uint8_t>(" + str(round(duration)) + ")}, "
notes_final = notes_final[:-2] + "}"

print(notes_final)
print(f"Length of sound: {len(new_notes)}")