import numpy as np

def read_data(file_path):
    ranges = []
    angles = []
    samples = []
    
    with open(file_path, 'r') as file:
        for line in file:
            # Split the line into range and angle
            parts = line.strip().split(',')
            if len(parts) == 3:
                try:
                    range_value = float(parts[0].strip())
                    angle_value = float(parts[1].strip())
                    sample_value = float(parts[2].strip())
                    ranges.append(range_value)
                    angles.append(angle_value)
                    samples.append(sample_value)
                except ValueError:
                    print(f"Warning: Skipping invalid line: {line.strip()}")
    return ranges, angles, samples

def main():
    file_path = '../LidarDump.txt'  # Change this to the path of your text file
    ranges, angles, samples = read_data(file_path)
    
    if ranges:
        print(f"Variance of ranges: {np.c(ranges, ddof = 1)}")
        print(f"Average of ranges: {np.mean(ranges)}")
    else:
        print("No valid range data to calculate variance.")
    
    if angles:
        print(f"Variance of angles: {np.var(angles, ddof=1)}")
        print(f"Average of angles: {np.mean(angles)}")

    else:
        print("No valid angle data to calculate variance.")

    if samples:
        print(f"Variance of number of samples: {np.var(samples, ddof=1)}")
        print(f"Average number of samples: {np.mean(samples)}")

    else:
        print("No valid data to calculate numbers of samples mean.")

if __name__ == "__main__":
    main()
