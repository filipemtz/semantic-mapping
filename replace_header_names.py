import os
import glob
import re

# Define the root directory
root_dir = './'

# Iterate over all files in the directory and its subdirectories
for root, dirs, files in os.walk(root_dir):
    for file in files:
        # Check if the file is a .h or .c file
        if file.endswith('.h') or file.endswith('.c') or file.endswith('.cpp'):
            print(file)
            # Create a file path to the file
            file_path = os.path.join(root, file)

            # Read in the contents of the file
            with open(file_path, 'r') as f:
                file_contents = f.read()

            # Find all #include statements in the file
            includes = re.findall(
                r'#include\s+<carmen/\w+(?:_\w+)*\.h>', file_contents)

            # Replace each #include statement with the path to the header file
            for include in includes:
                print("include:", include)

                # extract the name of the header file from the include line
                name = include.strip().split('/')[-1][:-1]
                print("name:", name)

                # Search for the header file using glob
                header_file_path = glob.glob(os.path.join(
                    root_dir, '**', name), recursive=True)

                print("header_file_path:", header_file_path)

                if header_file_path:
                    # Replace the #include statement with the new path
                    file_contents = re.sub(include, '#include "{}"'.format(
                        os.path.relpath(header_file_path[0], root_dir)), file_contents)

            # Write the new file contents back to the file
            with open(file_path, 'w') as f:
                f.write(file_contents)
