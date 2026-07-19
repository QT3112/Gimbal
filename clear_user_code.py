import re

def clear_user_code(file_path):
    with open(file_path, 'r') as f:
        content = f.read()

    # Regular expression to match everything between USER CODE BEGIN xxx and USER CODE END xxx
    # We want to match: /* USER CODE BEGIN [something] */\n[anything]\n/* USER CODE END [something] */
    # and replace with: /* USER CODE BEGIN [something] */\n/* USER CODE END [something] */
    
    # Non-greedy match for everything inside the block
    pattern = r'(/\* USER CODE BEGIN .*? \*/\n)(.*?)(\n?/\* USER CODE END .*? \*/)'
    
    def replacer(match):
        return match.group(1) + match.group(3)

    new_content = re.sub(pattern, replacer, content, flags=re.DOTALL)

    with open(file_path, 'w') as f:
        f.write(new_content)

    print("Cleared user code.")

clear_user_code('Core/Src/main.c')
