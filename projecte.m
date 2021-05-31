% Oleguer Gregori - 1566780
% Oriol Prat - 1565096


% Variables a modificar (si es modifiquen pot ser que funcioni pitjor):
dataset="streetLight/input"; %dataset a utilitzar
max_disappear=10; %nº de frames consecutius que fa que no trobem el vehicle per eliminar-lo
max_distance=30; %distància en frames a la que es buscarà el vehicle per fer tracking
ppm=320/30; %mida del frame en píxels / longitud real en metres d'aquests mateixos píxels


% 1. Creem el set d'imatges que utilitzarem més endavant
files = dir(dataset);
% Frames que es volen utilitzar (ES POT MODIFICAR)
ImageSet = files(501:850);
mides=imread(dataset+"/"+ImageSet(1).name);

% 2. Creem el vídeo amb color
video = VideoWriter("highwayVideo.avi");
open(video);
for ii=1:length(ImageSet)
    im = ImageSet(ii).name;
    I = imread(dataset+"/"+im);
    writeVideo(video,I);
end
close(video);

% 3.Creem la zona d'interès (ROI)
figure()
imshow(mides)
roi = drawpolygon('LineWidth',3,'Color','cyan');
coord=roi.Position;

% 4. Detecció d'objectes
Video = VideoReader("highwayVideo.avi");
fps=Video.FrameRate;

% Creem el detector d'objectes
Obj_detector = vision.ForegroundDetector("NumGaussians", 3, "NumTrainingFrames", 50);

% Creem els Bounding Box
Elemento_estructurante = strel("square", 3);
Bounding_box = vision.BlobAnalysis("BoundingBoxOutputPort", true, ... 
    "AreaOutputPort", false, "CentroidOutputPort", false, ... 
    "MinimumBlobArea", 150);
Video_highway = vision.VideoPlayer("Name", "Cotxes detectats");
video_highway.Position(3:4) = [650, 400];

% 5. Afegim els Bounding Box als frames si es troben dins la ROI i
% fem tracking als vehicles
vid = VideoWriter("detectedHighwayVideo.avi");
open(vid);

tracks=struct("id",[],"bbox",[],"count",[]);
vehicles=containers.Map(0,[0 0 [0 0] [0 0]]);
num_frame=0;
min_tracks=[];
count=1;
speeds=[];
while hasFrame(Video)
    % Per si volem veure els frames més lentament (ES POT MODIFICAR)
    %pause(0.05);
    
    num_frame=num_frame+1;
    
    % Llegim el frame i li passem al detector d'objectes
    frame = readFrame(Video);
    object = step(Obj_detector, frame);
    
    % Traiem el soroll de la imatge
    object_sense_soroll = imopen(object, Elemento_estructurante);
    
    % Obtenim les coordenades que representen l'objecte
    Box = step(Bounding_box, object_sense_soroll);
    
    b1=size(Box,1);
    % Si hi ha vehicles dins el ROI
    if b1>0
        % Eliminem els objectes que fa (max_disappear) frames que no apareixen
        for m=1:size(min_tracks)
            tracks(m).count=0;
        end
        for x=1:size(struct2table(tracks),1)
            tracks(x).count=tracks(x).count+1;
        end
        for x=1:size(struct2table(tracks),1)
            for y=1:size(struct2table(tracks),1)
                if tracks(y).count>=max_disappear
                    tracks(y)=[];
                    break
                end
            end
        end
        
        min_tracks=[];
        b3=0;
        % Per cada vehicle que ens ha detectat en el frame actual
        for i=1:b1
            % Calculem la posició mitja dels vehicles
            B1 = double(Box(i)+Box(b1*2+i)/2);
            B2 = double(Box(b1+i)+Box(b1*3+i)/2);
            
            % Només ens fixem en els vehicles que es troben dins el ROI
            if inROI(roi,B1,B2)==1
                b3=b3+1;
                s=size(struct2table(tracks),1);
                
                % Afegim els valors dels vehicles a l'array
                for l=1:s
                    if tracks(l).id==count
                        count=count+1;
                        break
                    end
                end
                tracks(s+1).id=count;
                tracks(s+1).bbox=[B1, B2];
                tracks(s+1).frame=num_frame;
                tracks(s+1).count=0;
                
                entra="False";
                % En el primer frame no podem fer tracking perquè no hi ha
                % frames anteriors
                if num_frame>1
                    min_distance=size(mides,2);
                    for z=1:s
                        % Calculem les distàncies entre vehicles de frames
                        % propers per determinar quins son el mateix
                        % vehicle en diferents frames (tracking)
                        if num_frame-1>=tracks(z).frame && tracks(z).frame>=num_frame-10
                            x=tracks(s+1).bbox(1)-tracks(z).bbox(1);
                            y=tracks(s+1).bbox(2)-tracks(z).bbox(2);
                            d=sqrt(x^2+y^2);
                            if d<min_distance && d<max_distance
                                entra="True";
                                min_distance=d;
                                min_track=z;
                            end
                        end
                    end
                end
                % Si hem aconseguit fer tracking en el vehicle actual
                if entra=="True"
                    obj=vehicles(tracks(min_track).id);
                    vehicles(tracks(min_track).id)=[obj(1), num_frame, [obj(3), obj(4)], tracks(min_track).bbox];
                    
                    % Calculem la velocitat instantània del vehicle des del
                    % primer frame fins el frame actual
                    box=tracks(min_track).bbox;
                    frames=obj(2)-obj(1);
                    seconds=frames/fps;
                    x=box(1)-obj(3);
                    y=box(2)-obj(4);
                    d=sqrt(x^2+y^2);
                    distance=d/ppm;
                    speed=(distance/1000)/(seconds/3600);
                    
                    tracks(s+1).id=tracks(min_track).id;
                    min_tracks(length(min_tracks)+1)=min_track;
                    if ~isnan(speed)
                        % Comprovem si els vehicles van més ràpids que la
                        % mitjana o menys
                        speeds(length(speeds)+1)=speed;
                        mitjana=mean(speeds);
                        frame = insertText(frame, [box(1), box(2)], speed, 'BoxOpacity', 1, 'FontSize', 10);
                        
                        % Si són més ràpids que la mitjana es veuen vermells,
                        % sinó verds i si és el primer frame en el que
                        % apareix o té velocitat nul·la es veu blau
                        if mitjana>=speed
                            frame = insertShape(frame, "Rectangle", Box(i,:),"Color", "green");
                        else
                            frame = insertShape(frame, "Rectangle", Box(i,:),"Color", "red");
                        end
                    else
                        frame = insertShape(frame, "Rectangle", Box(i,:),"Color", "blue");
                    end
                else
                    count=count+1;
                    vehicles(tracks(s+1).id)=[tracks(s+1).frame, tracks(s+1).frame, tracks(s+1).bbox, tracks(s+1).bbox];
                    frame = insertShape(frame, "Rectangle", Box(i,:),"Color", "blue");
                end
                box=tracks(s+1).bbox;
                frame = insertText(frame, [box(1)-15,box(2)], tracks(s+1).id, 'BoxOpacity', 1, 'FontSize', 10);
            end
        end
        % Afegim en el video el nombre de vehicles per frame
        frame = insertText(frame, [10 10], b3, 'BoxOpacity', 1, 'FontSize', 14);
        
        % Mostrem el frame amb els canvis que hem fet anteriorment
        writeVideo(vid,frame);
        step(Video_highway, frame);
    else
        % Si no hi ha vehicles dins el ROI mostrem el frame igualment
        frame=readFrame(Video);
        writeVideo(vid,frame);
        step(Video_highway,frame);
    end
end
close(vid);

% 6. Calculem les velocitats dels vehicles
k=keys(vehicles);
v=values(vehicles);
velocitats={};
for i=1:length(keys(vehicles))
    f=v{i};
    frames=f(2)-f(1);
    if frames>3
        seconds=frames/fps;
        x=f(5)-f(3);
        y=f(6)-f(4);
        d=sqrt(x^2+y^2);
        distance=d/ppm;
        speed=(distance/1000)/(seconds/3600);
        velocitats=[velocitats, [k{i} speed]];
    end
end

% Calculem la velocitat màxima i la velocitat mitjana
max_speed=0;
vel_mitjana=[];
for i=1:length(velocitats)
    vel=velocitats{i};
    if vel(2)>max_speed
       max_speed=vel(2);
       max_vehicle=vel(1);
    end
    vel_mitjana(length(vel_mitjana)+1)=vel(2);
end
mitjana_carretera=mean(vel_mitjana);
max_speed_round=round(max_speed,-1);

% Calculem l'histograma de velocitats mitjanes dels vehicles
% (si apareix algun valor igual a 0 és perquè eliminem els vehicles que apareixen
% durant molt pocs frames i la seva velocitat resultant és molt inferior a la real)
x = zeros(numel(velocitats), 1, "int8");
y = zeros(numel(velocitats), 1, "int8");
for u=1:numel(velocitats)
   x(u) = velocitats{u}(1);
   y(u) = (velocitats{u}(2));
end
bar(x,y,"barwidth",0.4)

fprintf("La velocitat mitjana dels vehicles és de %.2f km/h\n",mitjana_carretera);
fprintf("La velocitat màxima de la carretera és de %d km/h\n",max_speed_round);
fprintf("Per saber-ho ens basem en que la velocitat mitjana màxima ha estat de %.2f km/h, feta pel vehicle %d\n",max_speed,max_vehicle);
