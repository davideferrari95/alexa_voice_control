#Oggetti presenti in tutte e due le aree
area_destra = {
     'sale': ['il sale grosso', 21], 
     'pasta' : ['gli spaghetti', 32],
     'pomodoro': ['il pomodori pelati', 12], 
     'coltello': ['il coltello', 50],
     'forchetta': ['la forchetta di legno', 42],
     'bicchiere': ['il bicchiere bianco', 61]
 }
                 
area_sinistra = {
     'sale ': ['il sale iodato', 22], 
     'pasta': ['le pennette', 31],
     'pomodoro': ['la passata', 11], 
     'forchetta':['la forchettetta di plastica', 41],
     'bicchiere': ['il bicchiere trasparente',62]
                     }


 #Oggetti generici
oggettiGenerici = {'il pomodoro': 10, 
                   'il sale': 20, 
                   'la pasta': 30,
                   'la forchetta': 40,
                   'il coltello': 50,
                   'il bicchiere': 60,
                                }

#init variabili per determinare le dict 
gripper_aperto = 100
gripper_chiuso = 0
gripper_70 = 70

   #init dizionario oggetti
oggetti={
            
            "gli spaghetti":[
            [-1.16205341020693, -1.2011259359172364, 2.1040218512164515, -2.4113766155638636, -1.605603043233053, -2.687268082295553],
            gripper_aperto,
            gripper_chiuso,
            ],

            "la passata":[
            [-0.9700844923602503, -1.1493733686259766, 1.9297311941729944, -2.381669660607809, -1.6051719824420374, -2.463334862385885],
            gripper_chiuso,
            gripper_70,
            ]
                    }


# for key, value in area_destra.items():
#     if value[1] == value_to_find:
#         print("Chiave trovata nella dictionary 'self.area_destra':", value[0])
#         break

# for key, value in area_sinistra.items():
#     if value[1] == value_to_find:
#         print("Chiave trovata nella dictionary 'self.area_sinistra':", value[0])
#         break


# for key, value in oggettiGenerici.items():

#     if value == value_to_find:

#         print("Chiave trovata nella dictionary 'self.oggettiGenerici':", key)
#         break

value_to_find = (22,)

print(type(value_to_find))
print(type(value_to_find[0]))



# valore_da_trovare = 21


# for oggetto, dettagli in area_destra.items():
#     if dettagli[1] == valore_da_trovare:
#         nome_oggetto = oggetto
#         tupla_float = oggetti[nome_oggetto][0]
#         print("Tupla di float corrispondente:", tupla_float)
#         break
# else:
#     print("Il valore", valore_da_trovare, "non è presente nell'area destra.")

# def get_nome_corrispondente(valore, dizionario):
#     for nome, dettagli in dizionario.items():
#         if dettagli[1] == valore:
#             print(dettagli[0])
#             return dettagli[0]
#     return None

# valore_da_trovare = 11
# # nome_corrispondente = get_nome_corrispondente(valore_da_trovare, area_sinistra)

# if nome_corrispondente:
#     print("Nome corrispondente:", nome_corrispondente)
#     print(oggetti[nome_corrispondente][0])
# else:
#     print("Il valore", valore_da_trovare, "non è presente nel dizionario.")



