import os
import json
import sys

import os
import json

def borrar_claves_en_json(directorio_padre):
    for ruta_directorio, subdirectorios, archivos in os.walk(directorio_padre):
        for archivo in archivos:
            if archivo.endswith('.json'):
                ruta_archivo = os.path.join(ruta_directorio, archivo)
                
                # Leer el contenido del archivo JSON
                try:
                    with open(ruta_archivo, 'r') as f:
                        data = json.load(f)
                    
                    # Borrar "setPoints" y "localPositions" si existen
                    claves_a_borrar = ["setPoints", "localPositions"]
                    for clave in claves_a_borrar:
                        if clave in data:
                            del data[clave]
                            with open(ruta_archivo, 'w') as f:
                                json.dump(data, f, indent=4)
                    
                except Exception as e:
                    print(f"Error procesando {ruta_archivo}: {e}")

if __name__ == "__main__":
    borrar_claves_en_json("/tesis-aerogeneradores-ws/tests")
