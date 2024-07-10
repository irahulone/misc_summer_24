import cv2
import collections
import tkinter as tk

# Adjustable delay in seconds
adjustable_delay = 2  # Default delay value

# Open the webcam
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Could not open webcam.")
    exit()

# Get the frame rate of the webcam
fps = cap.get(cv2.CAP_PROP_FPS)

# Initialize GUI
root = tk.Tk()
root.title("Adjustable Delay GUI")

# Create a StringVar to hold the adjustable delay value
adjustable_delay_var = tk.StringVar(value=str(adjustable_delay))

# Create an entry widget for input
entry = tk.Entry(root, textvariable=adjustable_delay_var)
entry.pack()

# Calculate the initial number of frames to delay based on the delay time and frame rate
delay_frames = int(adjustable_delay * fps)

# Create a deque (double-ended queue) to store frames
frame_buffer = collections.deque(maxlen=delay_frames)

# Create a button to update the adjustable delay
def update_adjustable_delay():
    global adjustable_delay, delay_frames, frame_buffer
    try:
        new_delay = float(adjustable_delay_var.get())
        if new_delay > 0:
            adjustable_delay = new_delay
            delay_frames = int(adjustable_delay * fps)
            frame_buffer = collections.deque(maxlen=delay_frames)
            print(f"Updated adjustable delay to {adjustable_delay} seconds.")
        else:
            print("Please enter a positive number.")
    except ValueError:
        print("Invalid input. Please enter a number.")

button = tk.Button(root, text="Update Adjustable Delay", command=update_adjustable_delay)
button.pack()

# Main loop
while True:
    # Capture frame-by-frame
    ret, frame = cap.read()

    if not ret:
        print("Failed to grab frame")
        break

    # Append the current frame to the buffer
    frame_buffer.append(frame)

    # Display the live frame
    cv2.imshow('Live Feed', frame)

    # Display the delayed frame if the buffer has enough frames for the delay
    if len(frame_buffer) == delay_frames:
        delayed_frame = frame_buffer.popleft()
        cv2.imshow('Delayed Feed', delayed_frame)

    # Handle GUI events
    root.update_idletasks()
    root.update()

    # Press 'q' to exit the loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the capture and close windows
cap.release()
cv2.destroyAllWindows()
root.destroy()
