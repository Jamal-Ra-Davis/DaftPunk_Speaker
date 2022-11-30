print("Hello World")

def calc(x):
    ratio = 0.8
    val_exp = x**2.45 + 100
    val_lin = x*202 + 100
    return (ratio * val_exp) + ((1 - ratio) * val_lin)

def gen_array(array_name, size_name, size):
    output_str = "static uint16_t %s[%s] = {\n\t"%(array_name, size_name)
    for i in range(size):
        element = "%d, "%(int(calc(i)))
        output_str += element
        if i > 0 and i % 8 == 0:
            output_str += "\n\t"
    output_str += "\n};"
    print(output_str)


for i in range(40):
    print(i, ":", int(calc(i)))


gen_array("ranges", "FFT_BUCKETS", 40)
print(i, ":", int(calc(40)))


