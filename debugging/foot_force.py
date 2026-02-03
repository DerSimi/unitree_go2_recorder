
import sys
from data_loader import DataLoader

# Load data
data_loader = DataLoader(name=sys.argv[1])
data_loader.print_debug()

print(data_loader.get_foot_force())