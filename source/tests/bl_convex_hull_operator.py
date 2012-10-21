import sys

def test_tetrahedron():
    bpy.ops.object.add(type="MESH")
    data = bpy.context.active_object.data

    data.vertices.new(0, 0, 0, None)
        for idx, v in enumerate(m[0]):
            data.vertices[idx].co = v

def main():
    test_tetrahedron()
    sys.exit(1)

if __name__ == "__main__":
    main()

