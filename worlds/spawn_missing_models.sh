#!/bin/bash
# Script to spawn models that failed to load during world startup

echo "🔧 Spawning missing models in warehouse world..."

# Wait for Gazebo to be ready
sleep 2

# Spawn suitcase
echo "📦 Spawning suitcase..."
gz service -s /world/warehouse/create \
  --reqtype gz.msgs.EntityFactory \
  --reptype gz.msgs.Boolean \
  --timeout 5000 \
  --req 'sdf_filename: "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Suitcase1H", name: "suitcase", pose: {position: {x: 12.9, y: -5.69, z: 0.01}, orientation: {x: 0, y: 0, z: 0.9848, w: 0.1736}}'

# Spawn squareshelf
echo "📚 Spawning squareshelf..."
gz service -s /world/warehouse/create \
  --reqtype gz.msgs.EntityFactory \
  --reptype gz.msgs.Boolean \
  --timeout 5000 \
  --req 'sdf_filename: "https://fuel.gazebosim.org/1.0/OpenRobotics/models/SquareShelf", name: "squareshelf", pose: {position: {x: 14.25, y: -3.1, z: 0.01}, orientation: {x: 0, y: 0, z: 0.7986, w: 0.6018}}'

# Spawn minisofa
echo "🛋️  Spawning minisofa..."
gz service -s /world/warehouse/create \
  --reqtype gz.msgs.EntityFactory \
  --reptype gz.msgs.Boolean \
  --timeout 5000 \
  --req 'sdf_filename: "https://fuel.gazebosim.org/1.0/OpenRobotics/models/MiniSofa", name: "minisofa", pose: {position: {x: 13.256, y: -2.46, z: 0.01}, orientation: {x: 0, y: 0, z: 0.6157, w: 0.7880}}'

# Spawn sofa
echo "🛋️  Spawning sofa..."
gz service -s /world/warehouse/create \
  --reqtype gz.msgs.EntityFactory \
  --reptype gz.msgs.Boolean \
  --timeout 5000 \
  --req 'sdf_filename: "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Sofa", name: "sofa", pose: {position: {x: 13.1, y: -7.26, z: 0.01}, orientation: {x: 0, y: 0, z: 0.7071, w: 0.7071}}'

# Spawn table1
echo "☕ Spawning table1..."
gz service -s /world/warehouse/create \
  --reqtype gz.msgs.EntityFactory \
  --reptype gz.msgs.Boolean \
  --timeout 5000 \
  --req 'sdf_filename: "https://fuel.gazebosim.org/1.0/OpenRobotics/models/CoffeeTable", name: "table1", pose: {position: {x: 2, y: 1, z: 0.01}}'

echo "✅ Done! Check Gazebo to verify models appeared."
echo ""
echo "List of models in warehouse:"
gz model --list 2>&1 | grep -E "suitcase|squareshelf|minisofa|sofa|table1"
