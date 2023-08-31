#taken from https://patorjk.com/software/taag/#p=display&f=Cybermedium

ascii_numbers = r"""
____ _ _  _ ____                 
|___ | |  | |___                 
|    |  \/  |___                 

____ ____ _  _ ____              
|___ |  | |  | |__/              
|    |__| |__| |  \              

___ _  _ ____ ____ ____          
 |  |__| |__/ |___ |___          
 |  |  | |  \ |___ |___          

___ _ _ _ ____                   
 |  | | | |  |                   
 |  |_|_| |__|                   

____ _  _ ____                   
|  | |\ | |___                   
|__| | \| |___                   

____ ___ ____ ____ ___ ____ ___  
[__   |  |__| |__/  |  |___ |  \  |
___]  |  |  | |  \  |  |___ |__/  .
""".strip().split('\n\n')

if __name__ == '__main__':
    print(ascii_numbers)
    for num in ascii_numbers:
        print(num,'\n\n\n')
