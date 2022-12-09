import argparse
transformTemplate = '''
<node pkg="tf" type="static_transform_publisher" name="iris{id}_odom" args="{x} {y} 0 0 0 0 world iris{id}_odom 100"/>
<node pkg="tf" type="static_transform_publisher" name="iris{id}_top" args="0 0 0.41 0 0 0 iris{id} iris{id}_top 100"/>
<node pkg="tf" type="static_transform_publisher" name="iris{id}_front" args="0.06 0 0.35 0 1.5708 0 iris{id} iris{id}_front 100"/>
<node pkg="tf" type="static_transform_publisher" name="iris{id}_right" args="0 -0.06 0.35 0 1.5708 -1.5708 iris{id} iris{id}_right 100"/>
<node pkg="tf" type="static_transform_publisher" name="iris{id}_left" args="0 0.06 0.35 0 1.5708 1.5708 iris{id} iris{id}_left 100"/>
<node pkg="tf" type="static_transform_publisher" name="iris{id}_back" args="-0.06 0 0.35 0 -1.5708 3.1415 iris{id} iris{id}_back 100"/>



'''

endTF = '''
<node pkg="tf" type="static_transform_publisher" name="world_map" args="0.0 0.0 0 0 0 0 world map 100"/>
<node pkg="tf" type="static_transform_publisher" name="world_odom" args="0.0 0.0 0 0 0 0 world odom 100"/>
'''

def main():
    parser = argparse.ArgumentParser(description="rows = N \n cols = N")
    parser.add_argument('--rows', type=int, default=2)
    parser.add_argument('--cols', type=int, default=2)
    parser.add_argument('--path', type=str, default='transforms.launch')

    args = parser.parse_args()

    result = ""

    id = 0
    for i in range(args.rows):
        for j in range(args.cols):
            id += 1
            result += transformTemplate.format(id=id, x=i*0.75, y=j*0.75)

    
    output = "<launch>" + result + endTF + "</launch>"

    with open(args.path, 'w+') as f:
        f.write(output)

if __name__ == "__main__":
    main()