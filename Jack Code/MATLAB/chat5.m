function [prompt, response] = chat5(prompt)
    % Log file
    fid = fopen('logFile.log', 'w');
    
    % OpenAI API details
    apiKey = 
    promptMessage = 'Hello, ChatGPT! how are you?';
    endpoint = 'https://api.openai.com/v1/engines/curie/completions';
    
    % Format the curl command for OpenAI API
    commandStr = ['sudo curl -s -X POST ' endpoint ' ' ...
                  '-H "Content-Type: application/json" ' ...
                  '-H "Authorization: Bearer ' apiKey '" ' ...
                  '-d "{ \"prompt\": \"' promptMessage '\", \"max_tokens\": 150 }"'];
    % Convert string to char array
    commandChar = char(commandStr);
    
    % Execute the command on Raspberry Pi
    response = system(r, commandChar);
    
    % Log command
    fprintf(fid, '%s \n', commandChar);
    
    % Log result
    fprintf(fid, '%s \n', response);
   
end