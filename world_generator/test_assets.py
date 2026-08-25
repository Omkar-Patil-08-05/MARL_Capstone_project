from assets import AssetLibrary

lib = AssetLibrary()

print()

for k, v in lib.assets.items():

    print(k)

    print(len(v))

    print()