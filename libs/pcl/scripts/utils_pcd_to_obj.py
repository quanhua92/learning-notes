import os

DATASET_TXT_PATH = "D:\\mhmodels\\dataset_original.txt"
OUTPUT_TXT = "D:\\mhmodels\\dataset_objs.txt"
OUTPUT_DIR = "D:\\mhmodels\\models_obj"
TEMPLATE_OBJ = "end.obj"

list_data = [line.strip() for line in open(DATASET_TXT_PATH)]
template_lines = [line.strip() + "\n" for line in open(TEMPLATE_OBJ)]
print("template lines:" , len(template_lines))

total_data = len(list_data)
start_vertex_index = 11

output_txt_file = open(OUTPUT_TXT, "w")

for data_index, data_line in enumerate(list_data):
    if data_index == 0:
        continue
    pcd_path = data_line.split(" ")[-1]
    out_path = os.path.join(OUTPUT_DIR, pcd_path.split("\\")[-1])
    out_path = out_path.replace(".pcd", ".obj")
    lines = [line.strip() for line in open(pcd_path)]

    print("Write to ", out_path)
    output_file = open(out_path, "w")
    for index in range(start_vertex_index, len(lines), 1):
        cur_line = lines[index]
        if cur_line == "":
            break        
        vertices = cur_line.split(" ")
        output_line = "v {:.4f} {:.4f} {:.4f}\n".format(float(vertices[1]), float(vertices[2]), float(vertices[0]))
        output_file.write(output_line)
    output_file.writelines(template_lines)
    output_file.close()
    output_txt_file.write("{}\n".format(out_path))