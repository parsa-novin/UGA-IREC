from pyedb import Edb
import os
import sys

ansys_path = r"C:\Program Files\ANSYS Inc\ANSYS Student\v252\AnsysEM"

os.environ["ANSYSEM_ROOT252"] = ansys_path
os.environ["PATH"] += os.pathsep + ansys_path
sys.path.append(ansys_path)

edb = Edb(
    edbversion='2025.2',
    edbpath='EGSE2.xml',
    student_version=True
)

edb.save_edb()
edb.close_edb()