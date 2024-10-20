# Kiln controller plan

## Needs 
| Item | Location | Function | 
|------|----------|----------|
|**main**| kiln_controller.c| The main program that houses all sub functions and switch statements|
|------|----------|----------|
|**Hold Temp**|temp_hold.c|Function called on by main to hold the temp of the annealer for the days work|
|------|----------|----------|
|**Load Annealer**|door_open.c|Function called on by main to pause heating in order to allow safe loading and to prevent **Hold Temp** from damaging anything inside the kiln|
|------|----------|----------|
|8 Hour Anneal|8Hour|function to anneal the glass over 8 hours|
|------|----------|----------|
|12 Hour Anneal|12Hour|function to anneal the glass over 12 hours|
|------|----------|----------|
|18 Hour Anneal|18Hour|function to anneal the glass over 18 hours||------|----------|----------|
||||
|------|----------|----------|
||||
|------|----------|----------|