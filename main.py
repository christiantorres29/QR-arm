import cv2
from pyzbar import pyzbar
import serial
import numpy as np

# Initialize the Arduino communication
arduino = serial.Serial('/dev/ttyACM0', 9600)  # Replace 'COM3' with the correct port and baud rate

# Initialize the camera capture
cap = cv2.VideoCapture(0)

# Initialize a flag to track if the "Done" signal is received
done_signal_received = False

while True:
    # Read the frame from the camera
    ret, frame = cap.read()

    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect QR codes in the frame
    barcodes = pyzbar.decode(gray)

    # Process each detected QR code
    for barcode in barcodes:
        # Extract the bounding box coordinates of the QR code
        pts = np.array([barcode.polygon], np.int32)

        # Draw a green rectangle around the QR code
        cv2.polylines(frame, [pts], True, (0, 255, 0), 2)
        pts2=barcode.rect

        # Extract the data from the QR code
        barcode_data = barcode.data.decode("utf-8")
        barcode_type = barcode.type

        # Send the data to the Arduino via USB
        arduino.write((barcode_data+'\n').encode())  # Send the data as bytes

        # Display the QR code data on the frame
        cv2.putText(frame, barcode_data, (pts2[0], pts2[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # Display the frame
    cv2.imshow("QR Code Recognition", frame)

    # Check for the "Done" signal from the Arduino
    while arduino.in_waiting > 0:
        arduino_data = arduino.readline().decode().strip()  # Read the data from Arduino
        if arduino_data == "Done":
            done_signal_received = True

    # Exit the loop if the "Done" signal is received
    if done_signal_received:
        break

    # Wait for a key press and exit the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the Arduino and camera capture
arduino.close()
cap.release()
cv2.destroyAllWindows()