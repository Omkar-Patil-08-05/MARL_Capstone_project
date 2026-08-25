import re
with open('../worlds/earthquake_world.sdf', 'r') as f:
    content = f.read()

models = re.findall(r'<model name="([^"]+)">', content)
print(f"Total models: {len(models)}")
categories = {}
for m in models:
    cat = m.split('_')[0]
    if cat not in categories: categories[cat] = 0
    categories[cat] += 1

print("\nCategories:")
for k, v in sorted(categories.items()):
    print(f"{k}: {v}")
