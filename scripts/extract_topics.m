% relative path 
db_path = 'output_bags/surveyor_test1.bag/surveyor_test1.bag_0.db3'
output_file = 'output.csv'

system('sqlite3 output_bags/surveyor_test1.bag/surveyor_test1.bag_0.db3 ".tables"')

% Comando SQLite para exportar uma tabela para CSV
cmd = sprintf('sqlite3 %s ".headers on" ".mode csv" ".output %s" "SELECT id, topic_id, timestamp, hex(data) as data FROM messages;"', ...
              db_path, output_file);

% Executa o comando no terminal Linux
status = system(cmd);

% Verifica se o comando foi executado com sucesso
if status == 0
    disp('Dados exportados para CSV com sucesso!');
    
    % Lê o arquivo CSV gerado no MATLAB
    dados = readtable(output_file);
    disp(dados);
else
    error('Falha ao executar o comando SQLite.');
end

