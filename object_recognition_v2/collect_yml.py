import sys
import os
import shutil

location = 'classifiers'
location_data = ''
location_yml = 'hog'

if __name__ == '__main__':
    dirs = os.listdir(os.path.join(location, location_yml))
    file_list = open(os.path.join(location, 'list_yml.txt'), 'w')
    for i in dirs:
        if i.startswith('.'):
            continue
        file_list.write('{0}\tON\n'.format(os.path.abspath(os.path.join(location, location_yml, i))))
    file_list.close()
