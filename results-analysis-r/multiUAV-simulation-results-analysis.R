#!/usr/bin/Rscript
#
# CopterLogAnalysis.R - Statistical multicopter energy consumption
# analysis based on Ardupilot dataflash log files.
# Copyright (C) 2017  Thomas Dietrich
# 
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#

# Clear workspace
remove(list = ls())
# Close all devices still open from previous executions
graphics.off()

#####################################################################
# Logfiles to analyze ###############################################

#####################################################################
# Settings ##########################################################

logdata_folder <- "../results/"
tikzLocation = "./tikz/"
tikzLocation = "d:/ownCloudKainet/Dokumente/Promotion/Dissertation/tikz/"

# http://www.stat.columbia.edu/~tzheng/files/Rcolor.pdf
#graphCol1 <- "steelblue"
#graphCol2 <- "goldenrod"
#graphCol2_dark <- "goldenrod4"
#graphCol3 <- "gray70"

#graphCol1 <- "#353594"
graphCol1 <- "#4682B4" #steelblue
graphCol2 <- "#B8850A" #mygold
graphCol2_dark <- "goldenrod4"
graphCol3 <- "#B3B3B3" #mygray
graphCol3_dark <- "#737373"
graphCol3_darkdark <- "#434343"
graphCol4 <- "#780116" #myred

#####################################################################
# Install packages, load libraries ##################################

list.of.packages <- c("Hmisc", "geosphere", "ggplot2", "cowplot", "tikzDevice", "TTR", "xts", "forecast",
                      "data.table", "xtable", "stringr")
new.packages <- list.of.packages[!(list.of.packages %in% installed.packages()[,"Package"])]
if(length(new.packages)) install.packages(new.packages)
sapply(list.of.packages, require, character.only = TRUE)
lapply(list.of.packages, packageVersion)
remove(list = c("list.of.packages", "new.packages"))

# Increase max number of warnings
options(nwarnings=200) 

### Update packages (from time to time!)
#update.packages(checkBuilt=TRUE, ask=FALSE)

# plotlyUsername <- "user"
# plotlyApiKey <- "key"
#source("plotlyCredentials.R")

#Sys.setenv("plotly_username" = plotlyUsername)
#Sys.setenv("plotly_api_key" = plotlyApiKey)
#remove(plotlyUsername, plotlyApiKey)


theme_custom <- function () {
  theme_light() %+replace% 
    theme(
      panel.background  = element_blank(),
      axis.title = element_text(size = rel(0.8)),
      axis.ticks=element_blank(),
      legend.text = element_text(size = rel(0.8)),
      legend.title = element_text(size = rel(0.8)),
      panel.border=element_blank()
    )
}
theme_set(theme_custom())

options(tikzDefaultEngine = "xetex")

#####################################################################
# Simulation runtime estimation #####################################

{
  repetitions <- 16
  iterations.replM <- 1
  iterations.quant <- 3
  iterations.numUAVs <- 1
  hours <- 48
  
  birke_faktor <-(47 / 60) / 16 / (3 * 1 * 1 * 16)
  
  simulation_runtime_hours <- repetitions * iterations.replM * iterations.quant * iterations.numUAVs * hours * birke_faktor
  
  cat(paste("Expected runtime:", round(simulation_runtime_hours, digits = 1), "hours"))
}

#####################################################################

df.all <- data.frame()

for (filename in list.files(logdata_folder,"*.sca")) {
  logdata.file <- data.frame()
  cat(filename)
  cat(": ")
  filename.replM <- NA
  filename.quant <- NA
  filename.numUAVs <- NA
  
  filename.basename <- str_replace(filename, "(.*?)-.*", "\\1")
  
  filename.replM   <- ifelse(grepl("replM=", filename), str_replace(filename, ".*-replM=(\\d).*", "\\1"), NA)
  filename.quant   <- ifelse(grepl("quant=", filename), str_replace(filename, ".*-quant=(\\d).*", "\\1"), NA)
  filename.numUAVs <- ifelse(grepl("numUAVs=", filename), str_replace(filename, ".*-numUAVs=(\\d).*", "\\1"), NA)
  
  filename.repeat <- str_replace(filename, ".*-#(\\d+).*", "\\1")
  
  cat("Load File ... ")
  lines <- readLines(paste(logdata_folder, filename, sep=""))
  #head(lines)
  lines.scalar <- grep("scalar OsgEarthNet", lines, value=TRUE)
  #head(lines.scalar)
  
  cat("Parse Scalars ... ")
  for (line in lines.scalar) {
    node.type <- str_replace(line, "scalar OsgEarthNet\\.(.*?)\\[\\d+\\].*", "\\1")
    index <- str_replace(line, "scalar OsgEarthNet.*?\\[(\\d+)\\].*", "\\1")
    metric <- str_replace(line, "scalar OsgEarthNet.*?\\[\\d+\\] (\\w*).*", "\\1")
    value <-str_replace(line, "scalar OsgEarthNet.*?\\[\\d+\\] \\w* (.*)", "\\1")
    #print(paste(index, metric, value, sep = " --- "))
    
    logdata.line <- data.frame(
      basename =         as.factor(filename.basename),
      replM =            as.integer(filename.replM),
      quant =            as.integer(filename.quant),
      numUAVs =          as.integer(filename.numUAVs),
      simRun =           as.integer(filename.repeat),
      nodeType =         as.factor(node.type),
      index =            as.integer(index),
      metric =           as.factor(metric),
      value =            as.numeric(value)
    )
    logdata.file <- rbind(logdata.file, logdata.line)
  }
  cat("\n")
  df.all <- rbind(df.all, logdata.file)
}

# Noteworthy general data
head(df.all)
replMs <- unique(df.all$replM)
quant <- unique(df.all$quant)
numUAVs <- unique(df.all$numUAVs)

# Split types
df.all.uav <- subset(df.all, nodeType == 'uav')
df.all.cs <- subset(df.all, nodeType == 'cs')

#####################################################################

uavs_count <- max(df.all.uav$index)
sim_runs <- unique(df.all.uav$simRun)
metrics <- levels(factor(df.all.uav$metric))

cs_count <- max(df.all.cs$index)
metrics.cs <- levels(factor(df.all.cs$metric))


cat("Remove run+UAVs with 'fail' metric ... ")

uavs_all <- subset(df.all.uav, metric == 'utilizationFail')
uavs_all_failed <- subset(df.all.uav, metric == 'utilizationFail' & value != 0)

if (nrow(uavs_all_failed) > 0) {
  warning(paste(nrow(uavs_all_failed), "of", nrow(uavs_all), "UAVs over all simulation runs ended in 'fail' state", sep=" "))
}

if (nrow(uavs_all_failed) > 0) {
  for(i in 1:nrow(uavs_all_failed)) {
    row <- uavs_all_failed[i,]
    #print(row)
    df.all.uav <- df.all.uav[!(df.all.uav$replM==row$replM & df.all.uav$simRun==row$simRun & df.all.uav$index==row$index),]
  }
}
remove(uavs_all, uavs_all_failed)


simtimeSec <- 0
cat("Proofchecking Simtime ... ")
tolerance = 0.1
for (replMethod in replMs) {
  for (sim_run in sim_runs) {
    for (uav_index in 0:uavs_count) {
      df_subset <- subset(df.all.uav, replM == replMethod & simRun == sim_run & index == uav_index
                          & metric %in% c('utilizationSecMaintenance', 'utilizationSecMission', 'utilizationSecCharge', 'utilizationSecIdle')
      )
      if (nrow(df_subset) > 0) {
        #print(sum(df_subset$value))
        if (simtimeSec == 0) {
          simtimeSec = as.integer(0.5 + sum(df_subset$value))
          cat(paste("Simulation time:", simtimeSec))
        }
        else if (abs(simtimeSec - sum(df_subset$value)) > tolerance) stop("Simtime missmatch!")
      }
    }
  }
}

#####################################################################
#
#cat("Combining SimRuns ... ")
#df.comb <- data.frame()
#
#for (replMethod in replMs) {
#  for (index in 0:uavs_count) {
#    for (metric in metrics) {
#      df_subset <- subset(df.all.uav, replM == replMethod & index == index & metric == metric)
#      metric_value.mean <- sum(df_subset$value)
#      metric_value.stddev <- sqrt(var(df_subset$value))
#      #print(paste(replMethod, uav, metric, metric_value.mean, metric_value.stddev, sep = " -- "))
#      df <- data.frame(
#        basename =    levels(df_subset$basename),
#        replM =       as.integer(replMethod),
#        index =       as.integer(index),
#        metric =      as.factor(metric),
#        value =       as.numeric(metric_value.mean),
#        valueStddev = as.numeric(metric_value.stddev)
#      )
#      df.comb <- rbind(df.comb, df)
#    }    
#  }
#}
#####################################################################

cat("Summarizing Metrics per UAV ... ")
df.red.uav <- data.frame()

for (replMethod in replMs) {
  for (sim_run in sim_runs) {
    for (uav_index in 0:uavs_count) {
      df_subset <- subset(df.all.uav, replM == replMethod & simRun == sim_run & index == uav_index)
      df <- data.frame(
        basename =    levels(df_subset$basename),
        replM =       as.integer(replMethod),
        simRun =      as.integer(sim_run),
        index =       as.integer(uav_index),
        #
        secMission =                  as.numeric(subset(df_subset, metric == 'utilizationSecMission')$value),
        secMaintenance =              as.numeric(subset(df_subset, metric == 'utilizationSecMaintenance')$value),
        secCharge =                   as.numeric(subset(df_subset, metric == 'utilizationSecCharge')$value),
        secIdle =                     as.numeric(subset(df_subset, metric == 'utilizationSecIdle')$value),
        #
        energyMission =               as.numeric(subset(df_subset, metric == 'utilizationEnergyMission')$value),
        energyMaintenance =           as.numeric(subset(df_subset, metric == 'utilizationEnergyMaintenance')$value),
        energyCharge =                as.numeric(subset(df_subset, metric == 'utilizationEnergyCharge')$value),
        #
        energyOverdrawMission =       as.numeric(subset(df_subset, metric == 'utilizationEnergyOverdrawMission')$value),
        energyOverdrawMaintenance =   as.numeric(subset(df_subset, metric == 'utilizationEnergyOverdrawMaintenance')$value),
        #
        countMissions =               as.numeric(subset(df_subset, metric == 'utilizationCountMissions')$value),
        countManeuversMission =       as.numeric(subset(df_subset, metric == 'utilizationCountManeuversMission')$value),
        countManeuversMaintenance =   as.numeric(subset(df_subset, metric == 'utilizationCountManeuversMaintenance')$value),
        countChargeState =            as.numeric(subset(df_subset, metric == 'utilizationCountChargeState')$value),
        countOverdrawnAfterMission =  as.numeric(subset(df_subset, metric == 'utilizationCountOverdrawnAfterMission')$value),
        countIdleState =              as.numeric(subset(df_subset, metric == 'utilizationCountIdleState')$value)
      )
      df.red.uav <- rbind(df.red.uav, df)
    }
  }
}

cat("Summarizing Metrics per CS ... ")
df.red.cs <- data.frame()

for (replMethod in replMs) {
  for (sim_run in sim_runs) {
    for (cs_index in 0:cs_count) {
      df_subset <- subset(df.all.cs, replM == replMethod & simRun == sim_run & index == cs_index)
      df <- data.frame(
        basename =    levels(df_subset$basename),
        replM =       as.factor(replMethod),
        simRun =      as.integer(sim_run),
        index =       as.factor(cs_index),
        #
        usedPower =                as.numeric(subset(df_subset, metric == 'usedPower')$value),
        chargedPower =             as.numeric(subset(df_subset, metric == 'chargedPower')$value),
        chargedMobileNodes =       as.integer(subset(df_subset, metric == 'chargedMobileNodes')$value),
        chargedMobileNodesOthers = as.integer(sum(subset(df.all.cs, replM == replMethod & simRun == sim_run & metric == 'chargedMobileNodes')$value) - subset(df_subset, metric == 'chargedMobileNodes')$value),
        chargedMobileNodesAll =    as.integer(sum(subset(df.all.cs, replM == replMethod & simRun == sim_run & metric == 'chargedMobileNodes')$value)),
        reservations =             as.integer(subset(df_subset, metric == 'reservations')$value)
      )
      df.red.cs <- rbind(df.red.cs, df)
    }
  }
}

#####################################################################
# Independence CS/UAV ###############################################

df <- subset(df.red.uav, replM==0)

map_id_to_location <- function(id) c('\\ \\textbf{CS1:} Tennis court Ritzeb\\"uhl', '\\ \\textbf{CS2:} Sports field Manebach', '\\ \\textbf{CS3:} Hotel Gabelbach')[as.integer(id)]
starting_cs <- as.factor(map_id_to_location(1 + (df$index %% 3)))

secMission.quota <- 100 / (df$secMission + df$secMaintenance + df$secCharge) * df$secMission

tikz(paste(tikzLocation, "8_initial_location_plot_R.tex", sep = ""), standAlone=TRUE, timestamp = FALSE, width=5.99, height=2.0)

ggplot(df, aes(x = starting_cs, y = secMission / 60)) +
  geom_jitter(width=0.05, color=graphCol3_dark, alpha=0.3) +
  geom_boxplot(width=0.6, color=graphCol1, fill=alpha("white", 0.6), outlier.alpha=0) +
  coord_flip() +
  scale_x_discrete(limits = rev(levels(starting_cs))) +
  theme(axis.title.y = element_blank()) +
  labs(x="Charging Station", y="Time in Missions [min]")

dev.off()

df <- subset(df.red.cs, replM==0)

tikz(paste(tikzLocation, "8_cs_popularity_plot_R.tex", sep = ""), standAlone=TRUE, timestamp = FALSE, width=5.99, height=2.0)

ggplot(df, aes(x = map_id_to_location(index), y = chargedMobileNodes)) +
  geom_jitter(width=0.05, color=graphCol3_dark, alpha=0.6) +
  geom_boxplot(width=0.6, color=graphCol1, fill=alpha("white", 0.6), outlier.alpha=0) +
  coord_flip() +
  scale_x_discrete(limits = rev(levels(starting_cs))) +
  theme(axis.title.y = element_blank()) +
  labs(x="Charging Station", y="Served UAVs")

dev.off()


#####################################################################
# General Performance Data ##########################################
# only replM==0

df <- subset(df.red.uav, replM==0)
life_cycles_uav.mean <- mean(df$countMissions)
life_cycles_uav.stddev <- sqrt(var(df$countMissions))

cat(paste("Life cycles per UAV", life_cycles_uav.mean, life_cycles_uav.stddev))

cons_energy_per_uav.overall <- mean(df$energyMission + df$energyMaintenance)
cons_energy_per_uav.perhour <- mean(df$energyMission + df$energyMaintenance) / (simtimeSec / 3600)
cons_energy_per_uav.permission <- mean((df$energyMission + df$energyMaintenance) / df$countMissions)

cat(paste("Energy consumption per UAV", cons_energy_per_uav.overall, cons_energy_per_uav.perhour, cons_energy_per_uav.permission))


lifecycle_states <- c("Mission Execution", "Maintenance Flights", "Charge", "Idle")

ratios.time <- data.frame(
  class = "time",
  metric = as.factor(lifecycle_states),
  value = as.numeric(c(mean(df$secMission), mean(df$secMaintenance), mean(df$secCharge), mean(df$secIdle)))
)
ratios.time$metric2 <- factor(ratios.time$metric, rev(lifecycle_states))
ratios.time$percentage = round(100 * ratios.time$value/sum(ratios.time$value),digits=1)

ratios.energy <- data.frame(
  class = "energy",
  metric = as.factor(lifecycle_states),
  value = as.numeric(c(mean(df$energyMission), mean(df$energyMaintenance), mean(df$energyCharge), 0))
)
ratios.energy$metric2 <- factor(ratios.energy$metric, rev(lifecycle_states))
ratios.energy$percentage = round(100 * ratios.energy$value/sum(ratios.energy$value),digits=1)

#TEMPORARY
ratios <- ratios.time
ratios <- rbind(ratios.time, ratios.energy)


tikz(paste(tikzLocation, "8_life_cycle_time_ratio_plot_R.tex", sep = ""), standAlone=TRUE, timestamp = FALSE, width=5.9, height=1.3)

ggplot(ratios, aes(x=class, fill=metric2, y=percentage, label=percentage)) +
  geom_col() +
  #geom_text(color=graphCol3_darkdark, size=rel(3), position=position_stack(vjust=0.5)) +
  geom_text(size=rel(3), position=position_stack(vjust=0.5)) +
  coord_flip() +
  scale_x_discrete(limits = (levels(metric))) +
  theme(axis.text.y=element_blank()) +
  #theme(legend.position="bottom", legend.title = element_blank(), legend.justification = "left") +
  theme(legend.title=element_blank()) +
  #scale_fill_brewer(name="Life Cycle State", guide=guide_legend(reverse=TRUE)) +
  scale_fill_manual(name="Life Cycle State", guide=guide_legend(reverse=TRUE),
                    values=c(graphCol3, graphCol2, graphCol2_dark, graphCol1)) + 
  scale_y_continuous(breaks=seq(0, 100, 10)) +
  labs(x="Life Cycle State", y="Quota [\\%]")

dev.off()


#####################################################################
# Random Diagram Testing ############################################

# Test correlation with starting position (CS 0,1, or 2)
locations <- c('CS0 Tennis court Ritzebuehl', 'CS1 Sports field Manebach', 'CS2 Hotel Gabelbach')
starting_cs <- df.red.uav$index %% 3
starting_cs <- as.factor(locations[starting_cs + 1])


ggplot(df.red.uav, aes(starting_cs, countManeuversMission)) + geom_boxplot() + geom_jitter(width = 0.2)
ggplot(df.red.uav, aes(starting_cs, countIdleState)) + geom_boxplot() + geom_jitter(width = 0.2)
ggplot(df.red.uav, aes(starting_cs, secMission)) + geom_boxplot() + geom_jitter(width = 0.2)
ggplot(df.red.uav, aes(starting_cs, secIdle)) + geom_boxplot() + geom_jitter(width = 0.1)
ggplot(df.red.uav, aes(starting_cs, secMission + secMaintenance + secCharge)) + geom_boxplot() + geom_jitter(width = 0.1)

ggplot(df.red.uav, aes(starting_cs, 100 / (secMission + secMaintenance) * secMission)) + geom_boxplot() + geom_jitter(width = 0.1)
ggplot(df.red.uav, aes(starting_cs, 100 / (secMission + secMaintenance + secCharge) * secMission)) + geom_boxplot() + geom_jitter(width = 0.1)
ggplot(df.red.uav, aes(starting_cs, 100 / (secMission + secMaintenance + secCharge + secIdle) * secMission)) + geom_boxplot() + geom_jitter(width = 0.1)



# Statistics about CS
ggplot(df.red.cs, aes(replM, chargedMobileNodesAll)) + geom_boxplot() + geom_jitter(width = 0.1)
ggplot(df.red.cs, aes(index, chargedMobileNodes)) + geom_boxplot() + geom_jitter(width = 0.1)

ggplot(df.red.cs, aes(replM, chargedMobileNodesAll)) + geom_point()
ggplot(subset(df.red.cs, index==0 | index==1 | index==2), aes(replM, chargedPower)) + geom_point()


# Diagrams
ggplot(df.red.uav) +
  geom_bin2d(aes(x = 100 / (secMission + secMaintenance) * secMission, y = 100 / (secMission + secMaintenance) * secMaintenance)) +
  labs(x="Time in Mission", y="Time in Maintenance Flights")

ggplot(df.red.uav) +
  geom_bin2d(aes(x = 100 / (energyMission + energyMaintenance) * energyMission, y = 100 / (energyMission + energyMaintenance) * energyMaintenance)) +
  labs(x="Energy in Mission", y="Energy in Maintenance Flights")


ggplot(subset(df.red.uav, replM==0)) + geom_bin2d(aes(x = energyMission, y = energyMaintenance))
ggplot(subset(df.red.uav, replM==1)) + geom_bin2d(aes(x = energyMission, y = energyMaintenance))
ggplot(subset(df.red.uav, replM==2)) + geom_bin2d(aes(x = energyMission, y = energyMaintenance))

ggplot(subset(df.red.uav, replM==0)) + geom_bin2d(aes(x = 100 / (energyMission + energyMaintenance) * energyMission, y = 100 / (energyMission + energyMaintenance) * energyMaintenance)) + ylim(0, 100) + xlim(0, 100)
ggplot(subset(df.red.uav, replM==1)) + geom_bin2d(aes(x = 100 / (energyMission + energyMaintenance) * energyMission, y = 100 / (energyMission + energyMaintenance) * energyMaintenance)) + ylim(0, 100) + xlim(0, 100)
ggplot(subset(df.red.uav, replM==2)) + geom_bin2d(aes(x = 100 / (energyMission + energyMaintenance) * energyMission, y = 100 / (energyMission + energyMaintenance) * energyMaintenance)) + ylim(0, 100) + xlim(0, 100)




#####################################################################
#####################################################################
cat("F I N I S H E D")

