import fileinput
from glob import glob

label_string = "HY HP LSP LSR LEY LER LWY LH RSP RSR REY RER RWY RH LHYP LHR LHP LKP LAP LAR RHYP RHR RHP RKP RAP RAR Int Dur"


def reformat_all_mof():
    for mof_file in glob("mof/*.mof"):
        if "extern" in mof_file:
            continue
        print("start reformat: ", mof_file)
        label_string_added = False
        reformatted_label = None
        for line in fileinput.input(mof_file, inplace=True):
            reformatted_line = line
            tab_string = ["     "]
            if '\"' in line or 'motion_id' in line or 'label' in line or 'transition' in line:
                if len(line.split())>27:
                    continue
                reformatted_line = line
            elif 'hardness' in line:
                string_list = line.split()
                string_list = string_list[:1] + [string_ele.rjust(6) for string_ele in string_list[1:]]
                reformatted_line = ' '.join(string_list[:1] + [' '] + string_list[1:3] + tab_string + string_list[3:9]+ tab_string + string_list[9:15]+ tab_string + string_list[15:21]+ tab_string + string_list[21:27]+ tab_string + string_list[27:])
            else:
                if not line.isspace():
                    if not label_string_added:
                        label_string_added = True
                        string_list = label_string.split()
                        string_list = string_list[:1] + [string_ele.rjust(6) for string_ele in string_list[1:]]
                        reformatted_label = ' '.join(["\"             "] + string_list[:2] + tab_string + string_list[2:8]+ tab_string + string_list[8:14]+ tab_string + string_list[14:20]+ tab_string + string_list[20:26]+ tab_string + string_list[26:])
                    string_list = line.split()
                    string_list = [str(round(float(string_ele),1)) if string_ele not in ['*','-','2000','1000'] else string_ele for string_ele in string_list]
                    #This is formatting with additional unflipping mechanism  
                    #string_list = [(str(-1 * round(float(string_ele),1)) if index in [1,2,8,9,10,11,15,19] else str(round(float(string_ele),1))) if string_ele not in ['*','-','2000','1000'] else string_ele for index, string_ele in enumerate(string_list)]
                    string_list = [string_ele.replace(".0","").rjust(6) for string_ele in string_list]
                    reformatted_line = ' '.join(["          "] + string_list[:2] + tab_string + string_list[2:8]+ tab_string + string_list[8:14]+ tab_string + string_list[14:20]+ tab_string + string_list[20:26]+ tab_string + string_list[26:])
                else:
                    reformatted_line = "\n"
            if '\n' in reformatted_line:
                print('{}'.format(reformatted_line), end='')
            else:
                if reformatted_label:
                    print('{}\n{}'.format(reformatted_label, reformatted_line), end='\n')
                    reformatted_label = None
                else:
                    print('{}'.format(reformatted_line), end='\n')

                    
if __name__ == "__main__":
    reformat_all_mof()