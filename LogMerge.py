import os
import csv
import glob

vf_logs_path = r'C:\Users\18125\CRB15000-RL\Assets\VFLogs'
logs_path = r'C:\Users\18125\CRB15000-RL\Assets\Logs'
vf_logs_name = 'VF_log_env_*.csv'
logs_name = 'log_env_*.csv'

def merge_logs_to_csv(input_folder, output_file, max_rows=50000):
    try:
        # Normalize the input path
        input_folder = os.path.normpath(input_folder)
        
        # Determine which type of logs to process
        if input_folder == vf_logs_path:
            log_files = glob.glob(os.path.join(input_folder, vf_logs_name))
        else:
            log_files = glob.glob(os.path.join(input_folder, logs_name))
        
        # Check if any log files were found
        if not log_files:
            print(f"No matching log files found in {input_folder}.")
            return
        
        # Sort log files by name
        log_files.sort()
        
        with open(output_file, 'w', newline='', encoding='utf-8') as csvfile:
            csv_writer = csv.writer(csvfile, quoting=csv.QUOTE_ALL)
            
            # Write the header from the first log file
            with open(log_files[0], 'r', encoding='utf-8') as first_log:
                header = next(csv.reader(first_log))
                csv_writer.writerow(header)
            
            row_count = 0
            for log_file in log_files:
                with open(log_file, 'r', encoding='utf-8') as logfile:
                    csv_reader = csv.reader(logfile)
                    next(csv_reader)  # Skip the header
                    
                    # Write data rows
                    for row in csv_reader:
                        csv_writer.writerow(row)
                        row_count += 1
                        if row_count >= max_rows:
                            print(f"Reached the {max_rows} row limit. Stopping.")
                            return
        
        print(f"Merge completed. Output file: {output_file}")
        print(f"Total rows written: {row_count}")
    
    except Exception as e:
        print(f"An error occurred: {str(e)}")

# Usage example
merge_logs_to_csv(vf_logs_path, 'merged_vf_logs.csv')
merge_logs_to_csv(logs_path, 'merged_regular_logs.csv')